/*
 * sensor_types.h
 * Shared data structures used across every layer.
 *
 * ARCHITECTURE NOTE
 * ─────────────────
 * This header is the "contract" between layers:
 *
 *   HAL  →  Middleware  →  Sensor Processing  →  Data Pipeline  →  App
 *
 * Raw sensor readings flow UP through sensor_raw_t.
 * Processed / feature data flows UP through sensor_processed_t.
 * The StateMachine at the top consumes sensor_processed_t.
 */

#ifndef COMMON_SENSOR_TYPES_H_
#define COMMON_SENSOR_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════════
 *  RAW SENSOR DATA  (filled by Middleware layer)
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    /* IMU – BNO085 */
    float accel_x, accel_y, accel_z;       /* g          */
    float gyro_x,  gyro_y,  gyro_z;        /* deg/s      */
    float mag_x,   mag_y,   mag_z;         /* µT         */
    float lin_accel_x, lin_accel_y, lin_accel_z; /* g (gravity removed) */
    float roll, pitch, yaw;                /* degrees    */
    uint32_t total_steps;
    float    step_frequency;               /* Hz         */
    float    acc_variance;                 /* rolling variance of |accel| */

    /* Heart-rate – MAX30102 */
    uint32_t raw_ir;                       /* ADC counts */
    uint32_t heart_rate;                   /* BPM        */
    float    hrv;                          /* RMSSD ms   */

    /* Temperature – NTC */
    float body_temp;                       /* °C         */
    float ambient_temp;                    /* °C         */

    /* Battery */
    float batt_voltage;                    /* V          */
    float batt_percent;                    /* 0–100 %    */
} sensor_raw_t;

/* ═══════════════════════════════════════════════════════════════
 *  PROCESSED / DERIVED DATA  (filled by Sensor Processing layer)
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    float accel_magnitude;
    float gyro_magnitude;
    float mag_magnitude;
    float acc_variance;
    float range_of_motion;
    float respiration_rate;    /* breaths/min */
    float calorie_expenditure;
    bool  gait_asymmetry;
    time_t last_activity_time;
    time_t last_drink_time;
} sensor_processed_t;

/* ═══════════════════════════════════════════════════════════════
 *  ADC variable cache  (battery + temp bookkeeping)
 * ═══════════════════════════════════════════════════════════════ */
#define BATT_AVG_SAMPLES_MAX 16
typedef struct {
    float raw_volt_body;
    float raw_volt_amb;
    float raw_volt_battery;
    float batt_percent_buffer[BATT_AVG_SAMPLES_MAX];
    uint8_t batt_index;
    uint8_t batt_count;
    float   batt_percent;
    float   body_temp;
    float   amb_temp;
} adc_cache_t;

/* ═══════════════════════════════════════════════════════════════
 *  System time record
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    time_t   cloud_epoch_time;
    uint64_t curr_system_uptime_us;
    uint64_t last_reboot_time_us;
} system_time_t;

#endif /* COMMON_SENSOR_TYPES_H_ */
