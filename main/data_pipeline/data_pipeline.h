/*
 * data_pipeline.h
 *
 * ARCHITECTURE LAYER: Data Module + Logger
 * ────────────────────────────────────────────────────────────────
 * Combines raw + processed data into a unified snapshot, handles:
 *  - Session logging to SPIFFS (for later offline analysis)
 *  - JSON packet creation for BLE/WiFi streaming
 *  - Timestamping
 *
 * Input:  sensor_raw_t  +  sensor_processed_t  +  StateMachine state
 * Output: data_snapshot_t  (consumed by streaming module + app layer)
 */

#ifndef DATA_PIPELINE_H_
#define DATA_PIPELINE_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor_types.h"

/* ── Snapshot sent to streaming / app ────────────────────────── */
typedef struct {
    /* Identity */
    char     device_id[16];
    uint32_t packet_index;
    uint32_t timestamp_s;

    /* Vitals */
    uint32_t heart_rate;       /* BPM                */
    uint32_t raw_ir;           /* raw photodiode ADC */
    uint16_t hrv;              /* RMSSD ms           */
    uint16_t respiration_rate; /* breaths/min        */
    float    body_temp;        /* °C                 */
    float    ambient_temp;     /* °C                 */
    uint8_t  battery_pct;      /* 0–100 %            */

    /* Motion */
    float    accel_x, accel_y, accel_z;
    float    gyro_x,  gyro_y,  gyro_z;
    float    roll, pitch, yaw;
    uint32_t total_steps;
    float    step_frequency;
    float    acc_variance;

    /* Derived */
    float    range_of_motion;
    float    calorie_expenditure;

    /* Location (placeholder – no GPS yet) */
    float    latitude;
    float    longitude;
    uint16_t distance_m;

    /* Alerts */
    uint8_t  alert_code;
    uint8_t  alert_severity;
    uint16_t alert_value;

    /* State */
    uint8_t  activity_state;   /* StateType enum value */
    char     behaviour[32];
} data_snapshot_t;

/* ── API ─────────────────────────────────────────────────────── */

void data_pipeline_init(const char *device_id);

/**
 * @brief  Build a snapshot from current raw + processed data.
 *         Also appends one CSV row to the open log file (if recording).
 */
void data_pipeline_build_snapshot(const sensor_raw_t      *raw,
                                  const sensor_processed_t *proc,
                                  uint8_t                   activity_state,
                                  data_snapshot_t          *out);

/**
 * @brief  Serialise snapshot to a JSON string.
 * @param  snap  Input snapshot.
 * @param  buf   Output buffer.
 * @param  sz    Buffer size in bytes.
 * @return Pointer to buf, or NULL on error.
 */
char *data_pipeline_to_json(const data_snapshot_t *snap, char *buf, size_t sz);

/** Start / stop recording to SPIFFS. */
void data_pipeline_start_recording(const char *session_name);
void data_pipeline_stop_recording(void);

#endif /* DATA_PIPELINE_H_ */
