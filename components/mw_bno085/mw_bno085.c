/*
 * mw_bno085.c
 * Middleware – BNO085 IMU (SHTP over I2C).
 * Refactored from components/imu/bno085.c to match architecture.
 *
 * Responsibilities:
 *  - SHTP read/write
 *  - Feature enable commands
 *  - Packet parsing into sensor_raw_t
 *  - FFT-based respiration estimate
 *
 * Does NOT know about feature processing or state machines.
 */

#include "mw_bno085.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include <math.h>
#include <string.h>
#include <esp_dsp.h>

static const char *TAG = "MW_BNO085";

/* ── Device handle ───────────────────────────────────────────── */
static i2c_master_dev_handle_t s_dev = NULL;

/* ── Variance circular buffer ────────────────────────────────── */
static float  s_mag_buf[VAR_WINDOW_SIZE];
static uint16_t s_var_idx   = 0;
static uint16_t s_var_count = 0;
static float  s_acc_variance = 0.0f;

/* ── Step-frequency window ───────────────────────────────────── */
static uint32_t s_prev_steps       = 0;
static uint64_t s_step_win_start   = 0;
static float    s_step_frequency   = 0.0f;

/* ── Init ──────────────────────────────────────────────────────── */
void mw_bno085_init(i2c_master_dev_handle_t dev)
{
    s_dev = dev;
    ESP_LOGI(TAG, "BNO085 middleware ready");
}

/* ── SHTP transport ────────────────────────────────────────────── */
static esp_err_t shtp_write(uint8_t *data, size_t len)
{
    return i2c_master_transmit(s_dev, data, len, pdMS_TO_TICKS(1000));
}

static esp_err_t shtp_read(uint8_t *buf, size_t len)
{
    return i2c_master_receive(s_dev, buf, len, pdMS_TO_TICKS(1000));
}

/* ── Enable feature ────────────────────────────────────────────── */
void mw_bno085_enable_feature(uint8_t report_id, uint32_t interval_us)
{
    uint8_t cmd[] = {
        21, 0,
        2, 1,
        0xFD, report_id,
        0x00, 0x00, 0x00,
        (uint8_t)(interval_us & 0xFF),
        (uint8_t)((interval_us >> 8)  & 0xFF),
        (uint8_t)((interval_us >> 16) & 0xFF),
        (uint8_t)((interval_us >> 24) & 0xFF),
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    shtp_write(cmd, sizeof(cmd));
}

/* ── Parse helpers ─────────────────────────────────────────────── */
static void update_variance(float ax, float ay, float az)
{
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    s_mag_buf[s_var_idx] = mag;
    s_var_idx = (s_var_idx + 1) % VAR_WINDOW_SIZE;
    if (s_var_count < VAR_WINDOW_SIZE) s_var_count++;

    if (s_var_count == VAR_WINDOW_SIZE) {
        float sum = 0, sum_sq = 0;
        for (int i = 0; i < VAR_WINDOW_SIZE; i++) {
            sum    += s_mag_buf[i];
            sum_sq += s_mag_buf[i] * s_mag_buf[i];
        }
        float mean = sum / VAR_WINDOW_SIZE;
        s_acc_variance = (sum_sq / VAR_WINDOW_SIZE) - (mean * mean);
    }
}

static void parse_accel(uint8_t *p, sensor_raw_t *out)
{
    int16_t x = p[4] | (p[5] << 8);
    int16_t y = p[6] | (p[7] << 8);
    int16_t z = p[8] | (p[9] << 8);
    out->accel_x = x / 256.0f;
    out->accel_y = y / 256.0f;
    out->accel_z = z / 256.0f;
    /* variance computed in parse_lin_accel (gravity-free) — not here */
}

static void parse_gyro(uint8_t *p, sensor_raw_t *out)
{
    int16_t x = p[4] | (p[5] << 8);
    int16_t y = p[6] | (p[7] << 8);
    int16_t z = p[8] | (p[9] << 8);
    out->gyro_x = (x / 512.0f) * (180.0f / (float)M_PI);
    out->gyro_y = (y / 512.0f) * (180.0f / (float)M_PI);
    out->gyro_z = (z / 512.0f) * (180.0f / (float)M_PI);
}

static void parse_mag(uint8_t *p, sensor_raw_t *out)
{
    int16_t x = p[4] | (p[5] << 8);
    int16_t y = p[6] | (p[7] << 8);
    int16_t z = p[8] | (p[9] << 8);
    out->mag_x = x / 16.0f;
    out->mag_y = y / 16.0f;
    out->mag_z = z / 16.0f;
}

static void parse_lin_accel(uint8_t *p, sensor_raw_t *out)
{
    int16_t x = p[4] | (p[5] << 8);
    int16_t y = p[6] | (p[7] << 8);
    int16_t z = p[8] | (p[9] << 8);
    out->lin_accel_x = x / 256.0f;
    out->lin_accel_y = y / 256.0f;
    out->lin_accel_z = z / 256.0f;
    /* Variance uses LINEAR acceleration (gravity subtracted by BNO085),
     * so at rest the magnitude is ~0 m/s² and variance is ~0.
     * Raw accel at rest is ~9.8 m/s² (gravity) giving a falsely high baseline. */
    update_variance(out->lin_accel_x, out->lin_accel_y, out->lin_accel_z);
}

static void parse_game_rot(uint8_t *p, sensor_raw_t *out)
{
    int16_t i = p[4]  | (p[5]  << 8);
    int16_t j = p[6]  | (p[7]  << 8);
    int16_t k = p[8]  | (p[9]  << 8);
    int16_t r = p[10] | (p[11] << 8);
    float qi = i / 8192.0f, qj = j / 8192.0f;
    float qk = k / 8192.0f, qr = r / 8192.0f;
    float norm = sqrtf(qr*qr + qi*qi + qj*qj + qk*qk);
    if (norm > 0.0f) { qr /= norm; qi /= norm; qj /= norm; qk /= norm; }
    float sinp = 2.0f * (qr*qj - qk*qi);
    if (sinp >=  1.0f) sinp =  1.0f;
    if (sinp <= -1.0f) sinp = -1.0f;
    const float R2D = 57.2957795f;
    out->roll  = atan2f(2.0f*(qr*qi+qj*qk), 1.0f-2.0f*(qi*qi+qj*qj)) * R2D;
    out->pitch = asinf(sinp) * R2D;
    out->yaw   = atan2f(2.0f*(qr*qk+qi*qj), 1.0f-2.0f*(qj*qj+qk*qk)) * R2D;
}

static void parse_stepcount(uint8_t *p, sensor_raw_t *out)
{
    uint32_t steps = p[8] | (p[9] << 8);
    out->total_steps = steps;
    uint64_t now_ms = esp_timer_get_time() / 1000ULL;
    if (s_step_win_start == 0) s_step_win_start = now_ms;
    float dt = (now_ms - s_step_win_start) / 1000.0f;
    if (dt >= STEP_FREQ_UPDATE_WIN_SEC) {
        s_step_frequency = (steps - s_prev_steps) / dt;
        out->step_frequency = s_step_frequency;
        s_step_win_start = now_ms;
        s_prev_steps     = steps;
    } else {
        out->step_frequency = s_step_frequency;
    }
}

static uint8_t report_length(uint8_t id)
{
    switch (id) {
        case BNO_REPORT_ACCEL:      return 10;
        case BNO_REPORT_GYRO:       return 10;
        case BNO_REPORT_MAG:        return 10;
        case BNO_REPORT_LIN_ACCEL:  return 10;
        case BNO_REPORT_GAME_ROT:   return 12;
        case BNO_REPORT_STEPCOUNT:  return 12;
        case BNO_REPORT_HEART_RATE: return  6;
        case BNO_REPORT_GYRO_ROT:   return 14;
        default: return 0;
    }
}

static void parse_packet(uint8_t *p, uint16_t len, sensor_raw_t *out)
{
    uint8_t offset = 10;
    while (offset < len) {
        uint8_t rid  = p[offset - 1];
        uint8_t rlen = report_length(rid);
        if (rlen == 0 || (offset + rlen - 1) > len) return;
        switch (rid) {
            case BNO_REPORT_ACCEL:      parse_accel    (&p[offset-1], out); break;
            case BNO_REPORT_GYRO:       parse_gyro     (&p[offset-1], out); break;
            case BNO_REPORT_MAG:        parse_mag      (&p[offset-1], out); break;
            case BNO_REPORT_LIN_ACCEL:  parse_lin_accel(&p[offset-1], out); break;
            case BNO_REPORT_GAME_ROT:   parse_game_rot (&p[offset-1], out); break;
            case BNO_REPORT_STEPCOUNT:  parse_stepcount(&p[offset-1], out); break;
            default: return;
        }
        offset += rlen;
    }
}

/* ── Persistent accumulated frame — never zeroed between calls ─── */
static sensor_raw_t s_persistent_frame = {0};

/* Copy only the IMU fields that BNO085 owns into *out.
 * Never touch raw_ir / heart_rate / hrv (MAX30102) or temps (NTC). */
static inline void bno_copy_to(sensor_raw_t *out)
{
    out->accel_x       = s_persistent_frame.accel_x;
    out->accel_y       = s_persistent_frame.accel_y;
    out->accel_z       = s_persistent_frame.accel_z;
    out->gyro_x        = s_persistent_frame.gyro_x;
    out->gyro_y        = s_persistent_frame.gyro_y;
    out->gyro_z        = s_persistent_frame.gyro_z;
    out->mag_x         = s_persistent_frame.mag_x;
    out->mag_y         = s_persistent_frame.mag_y;
    out->mag_z         = s_persistent_frame.mag_z;
    out->lin_accel_x   = s_persistent_frame.lin_accel_x;
    out->lin_accel_y   = s_persistent_frame.lin_accel_y;
    out->lin_accel_z   = s_persistent_frame.lin_accel_z;
    out->roll          = s_persistent_frame.roll;
    out->pitch         = s_persistent_frame.pitch;
    out->yaw           = s_persistent_frame.yaw;
    out->total_steps   = s_persistent_frame.total_steps;
    out->step_frequency = s_persistent_frame.step_frequency;
    out->acc_variance  = s_persistent_frame.acc_variance;
}

/* ── Public get (called by raw_data_reader in LIVE mode) ─────── */
void mw_bno085_get_raw(sensor_raw_t *out)
{
    uint8_t header[BNO_HEADER_LEN];
    uint8_t payload[BNO_MAX_PAYLOAD];

    /* Read one SHTP packet — updates only the fields present in this packet.
       All other fields keep their last valid value from s_persistent_frame. */
    if (shtp_read(header, BNO_HEADER_LEN) != ESP_OK) {
        bno_copy_to(out);
        return;
    }

    uint16_t plen = ((uint16_t)header[0] | ((uint16_t)header[1] << 8)) & 0x7FFF;
    if (plen <= BNO_HEADER_LEN) {
        bno_copy_to(out);
        return;
    }
    if (plen > BNO_MAX_PAYLOAD) plen = BNO_MAX_PAYLOAD;

    if (shtp_read(payload, plen) != ESP_OK) {
        bno_copy_to(out);
        return;
    }

    /* Parse into the PERSISTENT frame — only updates fields that were
       present in this packet; everything else stays from last time */
    parse_packet(payload, plen, &s_persistent_frame);

    /* Always update variance from the circular buffer */
    s_persistent_frame.acc_variance = s_acc_variance;

    /* Copy only BNO-owned fields — leaves raw_ir/heart_rate/hrv/temps alone */
    bno_copy_to(out);
}
