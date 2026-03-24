/*
 * raw_data_reader.c
 *
 * Implements the switchable data source layer.
 * Sensor processing never calls middleware directly — it always
 * calls raw_data_reader_get(), which routes to live sensors OR
 * to file/UDP injection transparently.
 */

#include "raw_data_reader.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

/* ── Middleware headers (LIVE mode) ──────────────────────────── */
#include "mw_bno085.h"
#include "mw_max30102.h"
#include "mw_ntc.h"
#include "file_reader.h"

static const char *TAG = "RAW_DATA_READER";

/* ── Internal state ──────────────────────────────────────────── */
static raw_data_reader_cfg_t s_cfg;
static sensor_raw_t          s_udp_frame;
static bool                  s_udp_frame_ready = false;
static SemaphoreHandle_t     s_udp_mutex       = NULL;

/* ── Init ──────────────────────────────────────────────────────── */
void raw_data_reader_init(const raw_data_reader_cfg_t *cfg)
{
    memcpy(&s_cfg, cfg, sizeof(raw_data_reader_cfg_t));
    s_udp_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "Initialised — source: %s  override_mask: 0x%02X",
             raw_data_reader_source_name(cfg->source), cfg->override_mask);

    if (cfg->source == DATA_SRC_FILE) {
        ESP_LOGI(TAG, "Inject file: %s",
                 cfg->inject_file_path ? cfg->inject_file_path : "(none)");
    }
}

/* ── Runtime override ──────────────────────────────────────────── */
void raw_data_reader_set_override(uint8_t mask, data_source_t src)
{
    s_cfg.override_mask = mask;
    s_cfg.source        = src;
    ESP_LOGI(TAG, "Override updated — mask:0x%02X  src:%s",
             mask, raw_data_reader_source_name(src));
}

/* ── UDP push (called by UDP receive task) ─────────────────────── */
void raw_data_reader_push_udp_frame(const sensor_raw_t *frame)
{
    if (xSemaphoreTake(s_udp_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&s_udp_frame, frame, sizeof(sensor_raw_t));
        s_udp_frame_ready = true;
        xSemaphoreGive(s_udp_mutex);
    }
}

/* ── Internal: fill from external flash session ─────────────── */
static bool s_file_opened = false;

static bool fill_from_file(sensor_raw_t *out)
{
    if (!s_cfg.inject_file_path) return false;

    /* Open the session once, then call read_frame each tick */
    if (!s_file_opened) {
        if (file_reader_open_read(s_cfg.inject_file_path) != ESP_OK)
            return false;
        s_file_opened = true;
    }

    if (!file_reader_read_frame(out)) {
        /* EOF — loop back to start */
        file_reader_rewind();
        return file_reader_read_frame(out);
    }
    return true;
}

/* ── Internal: fill from UDP ─────────────────────────────────── */
static bool fill_from_udp(sensor_raw_t *out)
{
    bool ok = false;
    if (xSemaphoreTake(s_udp_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (s_udp_frame_ready) {
            memcpy(out, &s_udp_frame, sizeof(sensor_raw_t));
            ok = true;
        }
        xSemaphoreGive(s_udp_mutex);
    }
    return ok;
}

/* ── Internal: pick injected data for a single sensor group ─────── */
static bool fill_injected(sensor_raw_t *out)
{
    switch (s_cfg.source) {
        case DATA_SRC_FILE:   return fill_from_file(out);
        case DATA_SRC_UDP:    return fill_from_udp(out);
        case DATA_SRC_STATIC:
            memcpy(out, &s_cfg.static_frame, sizeof(sensor_raw_t));
            return true;
        default: return false;
    }
}

/* ── Public get ──────────────────────────────────────────────── */
bool raw_data_reader_get(sensor_raw_t *out)
{
    if (!out) return false;

    /* Do NOT memset to zero here — in LIVE mode, mw_bno085_get_raw()
     * maintains a persistent accumulated frame internally and copies it
     * into *out.  Zeroing would wipe the raw_ir / heart_rate fields that
     * mw_max30102_get_raw() wrote on the previous tick, breaking the
     * HR hold-last-value logic. */

    bool any_live = (s_cfg.override_mask != RAW_SRC_ALL);

    /* ── Live sensors ────────────────────────────────────────── */
    if (any_live) {
        /* MAX30102 first — reads FIFO directly, fast I2C, never blocks.
         * Must run before BNO so raw_ir is never clobbered by a memset. */
        if (!(s_cfg.override_mask & RAW_SRC_MAX30102))
            mw_max30102_get_raw(out);

        /* BNO085 — may block briefly waiting for SHTP packet */
        if (!(s_cfg.override_mask & RAW_SRC_BNO085))
            mw_bno085_get_raw(out);

        if (!(s_cfg.override_mask & RAW_SRC_NTC))
            mw_ntc_get_raw(out);
    }

    /* ── Injected sensors ────────────────────────────────────── */
    if (s_cfg.override_mask) {
        sensor_raw_t injected = {0};
        bool ok = fill_injected(&injected);

        if (ok) {
            if (s_cfg.override_mask & RAW_SRC_BNO085) {
                /* Copy only IMU fields */
                out->accel_x = injected.accel_x; out->accel_y = injected.accel_y;
                out->accel_z = injected.accel_z;
                out->gyro_x  = injected.gyro_x;  out->gyro_y  = injected.gyro_y;
                out->gyro_z  = injected.gyro_z;
                out->mag_x   = injected.mag_x;   out->mag_y   = injected.mag_y;
                out->mag_z   = injected.mag_z;
                out->lin_accel_x = injected.lin_accel_x;
                out->lin_accel_y = injected.lin_accel_y;
                out->lin_accel_z = injected.lin_accel_z;
                out->roll = injected.roll; out->pitch = injected.pitch;
                out->yaw  = injected.yaw;
                out->total_steps   = injected.total_steps;
                out->step_frequency = injected.step_frequency;
            }
            if (s_cfg.override_mask & RAW_SRC_MAX30102) {
                out->raw_ir     = injected.raw_ir;
                out->heart_rate = injected.heart_rate;
                out->hrv        = injected.hrv;
            }
            if (s_cfg.override_mask & RAW_SRC_NTC) {
                out->body_temp    = injected.body_temp;
                out->ambient_temp = injected.ambient_temp;
                out->batt_voltage = injected.batt_voltage;
                out->batt_percent = injected.batt_percent;
            }
        } else {
            ESP_LOGW(TAG, "Injected source returned no data");
            return false;
        }
    }

    return true;
}

/* ── Utility ─────────────────────────────────────────────────── */
const char *raw_data_reader_source_name(data_source_t src)
{
    switch (src) {
        case DATA_SRC_LIVE:   return "LIVE";
        case DATA_SRC_FILE:   return "FILE";
        case DATA_SRC_UDP:    return "UDP";
        case DATA_SRC_STATIC: return "STATIC";
        default:              return "UNKNOWN";
    }
}
