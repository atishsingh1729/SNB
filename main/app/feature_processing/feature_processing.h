/*
 * feature_processing.h
 *
 * ARCHITECTURE LAYER: Application / Feature Processing (top layer)
 * ────────────────────────────────────────────────────────────────
 * State machine that classifies dog behaviour and health states.
 * Consumes sensor_raw_t + sensor_processed_t.
 * Produces StateType + SubStates + alert_flags_t.
 *
 * UNCHANGED from original feature_processing.h —
 * only the include paths have been updated to the new structure.
 */

#ifndef FEATURE_PROCESSING_H_
#define FEATURE_PROCESSING_H_

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_timer.h"
#include "sensor_types.h"

/* ── Thresholds ──────────────────────────────────────────────── */
#define HR_HIGH                    120.0f
#define HR_REST_HIGH               110.0f
#define BODY_TEMP_HIGH              39.2f
#define AMBIENT_TEMP_HIGH           35.0f
#define ACTIVITY_LOW                25.0f
#define GAIT_IRREGULAR              60.0f
#define HRV_LOW                     20.0f
#define BREATH_RATE_HIGH            40.0f
#define NIGHT_MOTION_HIGH           60.0f

#define RESTING_HR_THRESHOLD        80.0f
#define HIGH_HR_THRESHOLD          160.0f
#define LOW_HR_THRESHOLD            60.0f
#define FEVER_THRESHOLD             39.2f
#define HYPOTHERMIA_THRESHOLD       36.66f
#define STEP_FREQ_WALKING_MIN        1.0f
#define STEP_FREQ_WALKING_MAX        2.0f
#define STEP_FREQ_RUNNING_MIN        2.0f
#define TREMOR_FREQ_MIN              5.0f
#define TREMOR_FREQ_MAX             12.0f
#define RESPIRATION_NORMAL_MIN      10.0f
#define RESPIRATION_NORMAL_MAX      30.0f
#define IDLE_TO_SLEEP_MINUTES        5
#define MOTION_STOP_SECONDS         10
#define LICKING_EXCESSIVE_SECONDS  120
#define DEHYDRATION_DURATION_MINUTES 30

#define GET_TIME_MS() (esp_timer_get_time() / 1000ULL)
#define TEMP_CONFIRM_MS   10000
#define RESP_CONFIRM_MS    8000
#define HRV_CONFIRM_MS    15000
#define HR_CONFIRM_MS     15000
#define DEHYD_CONFIRM_MS  60000   /* 1 min sustained before sub-state changes */

/* ── State enum ──────────────────────────────────────────────── */
typedef enum {
    STATE_RESTING_IDLE = 0,
    STATE_WALKING,
    STATE_RUNNING_PLAYING,
    STATE_SLEEPING,
    STATE_BARKING,
    STATE_TREMBLING_SHAKING,
    STATE_EPILEPSY,
    STATE_POST_SEIZURE,
    STATE_ARTHRITIS,
    STATE_PAIN_INJURY,
    STATE_FATIGUE_ALERT,
    STATE_STRESS_ANXIETY,
    STATE_EATING_DRINKING_IDLE,
    STATE_EATING_DRINKING_DETECTING,
    STATE_EATING_DRINKING_ACTIVE,
    STATE_EATING_DRINKING_COMPLETE,
    STATE_LICKING_SCRATCHING_IDLE,
    STATE_LICKING_SCRATCHING_GROOMING,
    STATE_LICKING_SCRATCHING_EXCESSIVE,
    STATE_LICKING_SCRATCHING_MEDICAL_ALERT,
    STATE_NO_DEHYDRATION,
    STATE_DEHYDRATION_DETECTED,
    STATE_DEHYDRATION_SEVERE,
    STATE_DEHYDRATION_CRITICAL,
    STATE_RESPIRATION_NORMAL,
    STATE_RESPIRATION_LOW,
    STATE_RESPIRATION_ELEVATED,
    STATE_TEMPERATURE_NORMAL,
    STATE_TEMPERATURE_FEVER,
    STATE_TEMPERATURE_HYPOTHERMIA,
    STATE_HRV_UNKNOW,
    STATE_HRV_STABLE,
    STATE_HRV_STRESSED,
    STATE_HR_UNKNOW,
    STATE_HR_STABLE,
    STATE_HR_HIGH,
    STATE_HR_LOW,
    STATE_UNKNOWN,
    STATE_MAXIMUM
} StateType;

/* ── Sub-states ──────────────────────────────────────────────── */
typedef struct {
    StateType eating_drinking,       eating_drinking_candidate;
    StateType licking_scratching,    licking_scratching_candidate;
    StateType dehydration,           dehydration_candidate;
    StateType respiration,           respiration_candidate;
    StateType temperature,           temperature_candidate;
    StateType hrv,                   hrv_candidate;
    StateType hr,                    hr_candidate;
    uint32_t  eating_drinking_timer, licking_scratching_timer;
    uint32_t  dehydration_timer,     respiration_timer;
    uint32_t  temperature_timer,     hrv_timer, hr_timer;
} SubStates;

/* ── SensorData (app view – mirrors sensor_raw_t + processed) ── */
typedef struct {
    /* IMU */
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float roll, yaw, pitch;
    float lin_accel_x, lin_accel_y, lin_accel_z;
    float magneto_x, magneto_y, magneto_z;
    uint32_t total_steps;
    float acc_variance, step_frequency;
    bool  gait_asymmetry;
    float range_of_motion;
    /* Vitals */
    uint32_t heart_rate;
    float    hrv;
    uint32_t rawIR;
    float    temperature;
    float    respiration_rate;
    /* Context */
    time_t last_activity_time, last_drink_time, state_entry_time;
    float  calorie_expenditure;
} SensorData;

/* ── State machine ───────────────────────────────────────────── */
typedef struct {
    StateType  current_state;
    SubStates  sub_states;
    time_t     state_start_time;
    SensorData sensor_data;
    float resting_heart_rate, normal_temp_min, normal_temp_max;
    int   idle_to_sleep_minutes;
    bool  was_active;
    float last_hrv, last_heart_rate;
} StateMachine;

extern StateMachine sm;

/* ── Alert flags ─────────────────────────────────────────────── */
typedef enum {
    ALERT_NONE              = 0,
    ALERT_DEHYDRATION       = (1 << 0),
    ALERT_ARTHRITIS         = (1 << 1),
    ALERT_FEVER             = (1 << 2),
    ALERT_HEART_DISTRESS    = (1 << 3),
    ALERT_ANXIETY           = (1 << 4),
    ALERT_RESPIRATORY       = (1 << 5),
    ALERT_PAIN              = (1 << 6),
    ALERT_OVERHEAT          = (1 << 7),
    ALERT_LETHARGY          = (1 << 8),
    ALERT_SLEEP_DISTURBANCE = (1 << 9),
} alert_flags_t;

/* ── API ─────────────────────────────────────────────────────── */
void initialize_state_machine(StateMachine *sm);
void process_state_transitions(StateMachine *sm);
void update_monitors(StateMachine *sm);
void show_state(StateMachine *sm);

/**
 * @brief  Copy raw + processed data into the StateMachine's SensorData.
 *         Call this once per tick before process_state_transitions().
 */
void feature_processing_feed(StateMachine          *sm,
                              const sensor_raw_t    *raw,
                              const sensor_processed_t *proc);

/* Helper predicates */
bool detect_tremor_pattern(SensorData *d);
bool detect_seizure_pattern(SensorData *d);
bool detect_head_down_posture(SensorData *d);
time_t get_state_duration(StateMachine *sm);

#endif /* FEATURE_PROCESSING_H_ */
