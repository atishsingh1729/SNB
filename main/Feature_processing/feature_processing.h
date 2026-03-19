#ifndef FEATURE_PROCESSING_H
#define FEATURE_PROCESSING_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_timer.h"

//---------------------------------------

#define HR_HIGH                120.0
#define HR_REST_HIGH           110.0
#define BODY_TEMP_HIGH         39.2
#define AMBIENT_TEMP_HIGH      35.0
#define ACTIVITY_LOW           25.0
#define GAIT_IRREGULAR         60.0
#define HRV_LOW                20.0
#define BREATH_RATE_HIGH       40.0
#define NIGHT_MOTION_HIGH      60.0


#define GET_TIME_MS() (esp_timer_get_time() / 1000ULL)
#define TEMP_CONFIRM_MS   10000
#define RESP_CONFIRM_MS    8000
#define HRV_CONFIRM_MS    15000
#define HR_CONFIRM_MS	  15000


	
typedef struct
{
    float heart_rate;      // BPM
    float body_temp;       // °C
    float ambient_temp;    // °C
    float activity_level;  // 0–100 (from IMU motion)
    float gait_variance;   // 0–100 (motion irregularity)
    float hrv;             // ms (heart rate variability)
    float breathing_rate; // breaths/min
    float night_motion;   // 0–100
} sensor_data_t;

extern sensor_data_t sensor_data;
typedef enum
{
    ALERT_NONE               = 0,
    ALERT_DEHYDRATION        = (1 << 0),
    ALERT_ARTHRITIS          = (1 << 1),
    ALERT_FEVER              = (1 << 2),
    ALERT_HEART_DISTRESS     = (1 << 3),
    ALERT_ANXIETY            = (1 << 4),
    ALERT_RESPIRATORY        = (1 << 5),
    ALERT_PAIN               = (1 << 6),
    ALERT_OVERHEAT           = (1 << 7),
    ALERT_LETHARGY           = (1 << 8),
    ALERT_SLEEP_DISTURBANCE  = (1 << 9)

} alert_flags_t;

extern alert_flags_t alert_flags;

uint32_t evaluate_alerts(sensor_data_t *d);
void process_sensor_stage(sensor_data_t *data);
void evaluate_alerts_function(void);


//---------------------------------------

// ====================
// Configuration
// ====================

#define RESTING_HR_THRESHOLD 			80.0f
#define HIGH_HR_THRESHOLD 				160.0f
#define LOW_HR_THRESHOLD 				60.0f
#define FEVER_THRESHOLD 				39.2f
#define HYPOTHERMIA_THRESHOLD 			36.66f
#define STEP_FREQ_WALKING_MIN 			1.0f
#define STEP_FREQ_WALKING_MAX 			2.0f
#define STEP_FREQ_RUNNING_MIN 			2.0f
#define TREMOR_FREQ_MIN 				5.0f
#define TREMOR_FREQ_MAX 				12.0f
#define RESPIRATION_NORMAL_MIN 			10.0f
#define RESPIRATION_NORMAL_MAX 			30.0f
#define IDLE_TO_SLEEP_MINUTES 			5
#define MOTION_STOP_SECONDS 			10
#define LICKING_EXCESSIVE_SECONDS 		120
#define DEHYDRATION_DURATION_MINUTES 	30

// Main state machine states
typedef enum {

    /* ================= MAIN STATES ================= */
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

    /* ================= EATING / DRINKING ================= */
    STATE_EATING_DRINKING_IDLE,          // nothing happening
    STATE_EATING_DRINKING_DETECTING,
    STATE_EATING_DRINKING_ACTIVE,
    STATE_EATING_DRINKING_COMPLETE,

    /* ================= LICKING / SCRATCHING ================= */
    STATE_LICKING_SCRATCHING_IDLE,       // normal rest state
    STATE_LICKING_SCRATCHING_GROOMING,
    STATE_LICKING_SCRATCHING_EXCESSIVE,
    STATE_LICKING_SCRATCHING_MEDICAL_ALERT,

    /* ================= HYDRATION ================= */
    STATE_NO_DEHYDRATION,
    STATE_DEHYDRATION_DETECTED,
    STATE_DEHYDRATION_SEVERE,
    STATE_DEHYDRATION_CRITICAL,

    /* ================= MONITOR STATES ================= */
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
	
	
    /* ================= SYSTEM ================= */
    STATE_UNKNOWN,
    STATE_MAXIMUM

} StateType;




// ====================
// Data Structures
// ====================

// Sensor data structure
typedef struct {
    // IMU data
    float accel_x, accel_y, accel_z;      // Acceleration in g
    float gyro_x, gyro_y, gyro_z;         // Gyroscope in rad/s
	float roll, yaw, pitch;         // Gyroscope in rad/s
	float lin_accel_x, lin_accel_y, lin_accel_z; // Linear Acceleration in g
	float magneto_x, magneto_y, magneto_z; 
	uint32_t total_steps;					//Steps from BNO inbuilt sensor fusion
    float acc_variance;                   // Variance of acceleration
    float step_frequency;                 // Step frequency in Hz
    bool gait_asymmetry;                  // Gait asymmetry flag
    float range_of_motion;                // ROM percentage
    
    // Audio data
    bool bark_detected;
    bool crunch_lap_sound;
    bool panting_detected;
    
    // Physiological data
    uint32_t heart_rate;                     // BPM
    float hrv;                            // Heart rate variability
	uint32_t rawIR;
    float temperature;                    // Celsius
    float respiration_rate;               // Breaths/min
    
    // Context data
    time_t last_activity_time;
    time_t last_drink_time;
    time_t state_entry_time;
    float calorie_expenditure;
} SensorData;

typedef struct {
	bool detected;	
	uint8_t confidence_score;
	uint32_t detected_time;
	// uint32_t configuration structure;
}state_attributes_t;

// Sub-state containers
typedef struct {
    /* -------- Active Sub-States -------- */
    StateType eating_drinking;
    StateType licking_scratching;
    StateType dehydration;
    StateType respiration;
    StateType temperature;
    StateType hrv;
	StateType hr;

    /* -------- Candidate States (for time filtering) -------- */
    StateType eating_drinking_candidate;
    StateType licking_scratching_candidate;
    StateType dehydration_candidate;
    StateType respiration_candidate;
    StateType temperature_candidate;
    StateType hrv_candidate;
	StateType hr_candidate;
    /* -------- Timers (ms timestamps) -------- */
    uint32_t eating_drinking_timer;
    uint32_t licking_scratching_timer;
    uint32_t dehydration_timer;
    uint32_t respiration_timer;
    uint32_t temperature_timer;
    uint32_t hrv_timer;
	uint32_t hr_timer;

} SubStates;

// Main state machine structure
typedef struct {
    StateType current_state;
    SubStates sub_states;
    time_t state_start_time;
    SensorData sensor_data;
    
    // Configuration
    float resting_heart_rate;
    float normal_temp_min;
    float normal_temp_max;
    int idle_to_sleep_minutes;
    
    // State persistence
    bool was_active;
    float last_hrv;
    float last_heart_rate;
} StateMachine;

extern StateMachine sm;
// ====================
// Function Declarations
// ====================

// State transition functions
bool check_resting_idle_to_walking(StateMachine* sm);
bool check_resting_idle_to_sleeping(StateMachine* sm);
bool check_resting_idle_to_barking(StateMachine* sm);
bool check_resting_idle_to_eating_drinking(StateMachine* sm);
bool check_resting_idle_to_trembling(StateMachine* sm);
bool check_walking_to_running(StateMachine* sm);
bool check_walking_to_resting(StateMachine* sm);
bool check_walking_to_pain_injury(StateMachine* sm);
bool check_walking_to_arthritis(StateMachine* sm);
bool check_running_to_fatigue(StateMachine* sm);
bool check_sleeping_to_epilepsy(StateMachine* sm);
bool check_sleeping_to_trembling(StateMachine* sm);
bool check_barking_to_stress(StateMachine* sm);
bool check_trembling_to_epilepsy(StateMachine* sm);
bool check_trembling_to_stress(StateMachine* sm);
bool check_arthritis_to_pain(StateMachine* sm);
bool check_pain_to_licking(StateMachine* sm);
bool check_stress_to_licking(StateMachine* sm);
bool check_critical_conditions(StateMachine* sm);

// State entry/exit handlers
void enter_state(StateMachine* sm, StateType new_state);
void exit_state(StateMachine* sm, StateType old_state);

// Helper functions
float calculate_imu_variance(SensorData* data);
bool detect_step_pattern(SensorData* data);
bool detect_tremor_pattern(SensorData* data);
bool detect_seizure_pattern(SensorData* data);
bool detect_head_down_posture(SensorData* data);
time_t get_state_duration(StateMachine* sm);
bool is_hrv_stable(SensorData* data, float threshold);

// Monitor functions
void update_monitors(StateMachine* sm);
void process_state_transitions(StateMachine* sm);
void show_state(StateMachine* sm);


void initialize_state_machine(StateMachine* sm);
#endif // FEATURE_PROCESSING_H