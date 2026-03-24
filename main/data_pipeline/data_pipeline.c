/*
 * data_pipeline.c
 * Data Module + Logger implementation.
 * Refactored from main/struct_2_json.c  +  main/sensor_data_processing/data_processing.c
 */

#include "data_pipeline.h"
#include "file_reader.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

static const char *TAG = "DATA_PIPELINE";

static char    s_device_id[16]   = "NECKBAND_01";
static uint32_t s_packet_index   = 0;
static bool     s_recording      = false;

/* ── Init ──────────────────────────────────────────────────────── */
void data_pipeline_init(const char *device_id)
{
    if (device_id) strncpy(s_device_id, device_id, sizeof(s_device_id) - 1);
    ESP_LOGI(TAG, "Data pipeline init — device: %s", s_device_id);
}

/* ── Build snapshot ─────────────────────────────────────────────── */
void data_pipeline_build_snapshot(const sensor_raw_t      *raw,
                                   const sensor_processed_t *proc,
                                   uint8_t                   activity_state,
                                   data_snapshot_t          *out)
{
    memset(out, 0, sizeof(data_snapshot_t));

    strncpy(out->device_id, s_device_id, sizeof(out->device_id) - 1);
    out->packet_index     = s_packet_index++;
    out->timestamp_s      = (uint32_t)time(NULL);

    /* Vitals */
    out->heart_rate        = raw->heart_rate;
    out->raw_ir            = raw->raw_ir;
    out->hrv               = (uint16_t)raw->hrv;
    out->respiration_rate  = (uint16_t)proc->respiration_rate;
    out->body_temp         = raw->body_temp;
    out->ambient_temp      = raw->ambient_temp;
    out->battery_pct       = (uint8_t)raw->batt_percent;

    /* Motion */
    out->accel_x           = raw->accel_x;
    out->accel_y           = raw->accel_y;
    out->accel_z           = raw->accel_z;
    out->gyro_x            = raw->gyro_x;
    out->gyro_y            = raw->gyro_y;
    out->gyro_z            = raw->gyro_z;
    out->roll              = raw->roll;
    out->pitch             = raw->pitch;
    out->yaw               = raw->yaw;
    out->total_steps       = raw->total_steps;
    out->step_frequency    = raw->step_frequency;
    out->acc_variance      = proc->accel_magnitude; /* magnitude as proxy */

    /* Derived */
    out->range_of_motion   = proc->range_of_motion;
    out->calorie_expenditure = proc->calorie_expenditure;

    /* State */
    out->activity_state    = activity_state;
    strncpy(out->behaviour, "NONE", sizeof(out->behaviour) - 1);

    /* Placeholder location */
    out->latitude  = 0.0f;
    out->longitude = 0.0f;
    out->distance_m = 0;

    /* Log to file if recording */
    if (s_recording) {
        file_reader_write_frame(raw);
    }
}

/* ── JSON serialisation ─────────────────────────────────────────── */
char *data_pipeline_to_json(const data_snapshot_t *snap, char *buf, size_t sz)
{
    cJSON *j = cJSON_CreateObject();
    if (!j) return NULL;

    cJSON_AddStringToObject(j, "id",      snap->device_id);
    cJSON_AddNumberToObject(j, "pkt",     snap->packet_index);
    cJSON_AddNumberToObject(j, "ts",      snap->timestamp_s);
    cJSON_AddNumberToObject(j, "bat",     snap->battery_pct);

    /* Vitals */
    cJSON_AddNumberToObject(j, "HR",      snap->heart_rate);
    cJSON_AddNumberToObject(j, "rawIR",   snap->raw_ir);
    cJSON_AddNumberToObject(j, "HRV",     snap->hrv);
    cJSON_AddNumberToObject(j, "RR",      snap->respiration_rate);
    cJSON_AddNumberToObject(j, "TEMP",    snap->body_temp);
    cJSON_AddNumberToObject(j, "AMB",     snap->ambient_temp);

    /* Motion */
    cJSON_AddNumberToObject(j, "ax",      snap->accel_x);
    cJSON_AddNumberToObject(j, "ay",      snap->accel_y);
    cJSON_AddNumberToObject(j, "az",      snap->accel_z);
    cJSON_AddNumberToObject(j, "gx",      snap->gyro_x);
    cJSON_AddNumberToObject(j, "gy",      snap->gyro_y);
    cJSON_AddNumberToObject(j, "gz",      snap->gyro_z);
    cJSON_AddNumberToObject(j, "roll",    snap->roll);
    cJSON_AddNumberToObject(j, "pitch",   snap->pitch);
    cJSON_AddNumberToObject(j, "yaw",     snap->yaw);
    cJSON_AddNumberToObject(j, "steps",   snap->total_steps);
    cJSON_AddNumberToObject(j, "stepHz",  snap->step_frequency);
    cJSON_AddNumberToObject(j, "var",     snap->acc_variance);
    cJSON_AddNumberToObject(j, "ROM",     snap->range_of_motion);

    /* State */
    cJSON_AddNumberToObject(j, "ACT",     snap->activity_state);
    cJSON_AddStringToObject(j, "beh",     snap->behaviour);

    /* Alerts */
    cJSON_AddNumberToObject(j, "alert",   snap->alert_code);
    cJSON_AddNumberToObject(j, "al_sev",  snap->alert_severity);
    cJSON_AddNumberToObject(j, "al_val",  snap->alert_value);

    char *raw_str = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);

    if (!raw_str) return NULL;
    strncpy(buf, raw_str, sz - 1);
    buf[sz - 1] = '\0';
    free(raw_str);

    return buf;
}

/* ── Recording control ──────────────────────────────────────────── */
void data_pipeline_start_recording(const char *session_name)
{
    if (file_reader_open_write(session_name) == ESP_OK) {
        s_recording = true;
        ESP_LOGI(TAG, "Recording started: %s", session_name);
    }
}

void data_pipeline_stop_recording(void)
{
    if (s_recording) {
        file_reader_close();
        s_recording = false;
        ESP_LOGI(TAG, "Recording stopped");
    }
}
