#include "feature_processing.h"
//#include "data_processing.h"
// #include "signal_list.h"
#include <string.h> 
#include <stdio.h>
//#include <math.h>

// static s_all_processed_signals_t signals;
// s_all_raw_signals_t raw_signals;

extern float step_frequency;
extern float acc_variance;

StateMachine sm;

state_attributes_t state_data[STATE_MAXIMUM-1] = {0};

// ====================
// State Transition Logic
// ====================

bool check_resting_idle_to_walking(StateMachine* sm) {
    // IMU rhythmic step pattern (1-3 Hz) + moderate acceleration
    if (sm->sensor_data.step_frequency >= STEP_FREQ_WALKING_MIN && 
        sm->sensor_data.step_frequency <= STEP_FREQ_WALKING_MAX &&
        sm->sensor_data.acc_variance > 1.0f) {
        return true;
    }
    return false;
}

bool check_resting_idle_to_sleeping(StateMachine* sm) {
    // >5 min idle + HR drop 10% + HRV increase
    time_t duration = get_state_duration(sm);

    if (duration >= IDLE_TO_SLEEP_MINUTES * 60) {
        float hr_drop = ((sm->last_heart_rate - sm->sensor_data.heart_rate) / 
                        sm->last_heart_rate) * 100.0f;
        bool hrv_increased = sm->sensor_data.hrv > sm->last_hrv;
        
        if (hr_drop >= 10.0f && hrv_increased) {
            return true;
        }
    }
    return false;
}

bool check_resting_idle_to_barking(StateMachine* sm) {
    return sm->sensor_data.bark_detected;
}

bool check_resting_idle_to_eating_drinking(StateMachine* sm) {
    // Head-down posture + IMU pattern
    if (detect_head_down_posture(&sm->sensor_data) && 
        sm->sensor_data.crunch_lap_sound) {
        return true;
    }
    return false;
}

bool check_resting_idle_to_trembling(StateMachine* sm) {
    // IMU 5-12Hz vibration + context check

    if (detect_tremor_pattern(&sm->sensor_data)) {
        // Context check: not during normal activity
        if (sm->current_state == STATE_RESTING_IDLE || 
            sm->current_state == STATE_SLEEPING) {
            return true;
        }
    }
    return false;
}

bool check_walking_to_running(StateMachine* sm) {
    // Step frequency > 3.5Hz + high acceleration variance
    if (sm->sensor_data.step_frequency > STEP_FREQ_RUNNING_MIN &&
        sm->sensor_data.acc_variance > 1.0f) {
        return true;
    }
    return false;
}

bool check_walking_to_resting(StateMachine* sm) {
    // Motion stops for >30 seconds
    time_t duration = get_state_duration(sm);
    if (duration >= MOTION_STOP_SECONDS && 
        sm->sensor_data.acc_variance < 0.1f) {
        return true;
    }
    return false;
}

bool check_walking_to_pain_injury(StateMachine* sm) {
    // Gait asymmetry + HRV drop
    if (sm->sensor_data.gait_asymmetry && 
        sm->sensor_data.hrv < sm->last_hrv * 0.8f) {
        return true;
    }
    return false;
}

bool check_walking_to_arthritis(StateMachine* sm) {
    // Reduced ROM + stiffness after rest
    if (sm->sensor_data.range_of_motion < 70.0f) {  // 70% of normal ROM
        time_t rest_duration = time(NULL) - sm->sensor_data.last_activity_time;
        if (rest_duration > 300) {  // 5 minutes since last activity
            return true;
        }
    }
    return false;
}

bool check_running_to_fatigue(StateMachine* sm) {
    // Sustained high activity + HR recovery delay
    time_t duration = get_state_duration(sm);
    if (duration > 600) {  // 10 minutes of high activity
        // Check if HR remains elevated despite reduced activity
        if (sm->sensor_data.heart_rate > RESTING_HR_THRESHOLD * 1.5f &&
            sm->sensor_data.acc_variance < 0.5f) {
            return true;
        }
    }
	
	
    return false;
}

bool check_sleeping_to_epilepsy(StateMachine* sm) {
    // Violent jerking + HR > 150 bpm
    if (detect_seizure_pattern(&sm->sensor_data) && 
        sm->sensor_data.heart_rate > HIGH_HR_THRESHOLD) {
        return true;
    }
    return false;
}

bool check_sleeping_to_trembling(StateMachine* sm) {
    // Small twitches + HR spikes (REM sleep)
    if (detect_tremor_pattern(&sm->sensor_data) &&
        sm->sensor_data.heart_rate > sm->last_heart_rate * 1.2f) {
        return true;
    }
    return false;
}

bool check_barking_to_stress(StateMachine* sm) {
    // Combined with high HR + low HRV
    if (sm->sensor_data.heart_rate > RESTING_HR_THRESHOLD * 1.3f &&
        sm->sensor_data.hrv < 50.0f) {  // Low HRV threshold
        return true;
    }
    return false;
}

bool check_trembling_to_epilepsy(StateMachine* sm) {
    // Pattern escalates to violent jerking
    if (detect_seizure_pattern(&sm->sensor_data)) {
        return true;
    }
    return false;
}

bool check_trembling_to_stress(StateMachine* sm) {
    // Combined with low HRV + barking
    if (sm->sensor_data.hrv < 50.0f && sm->sensor_data.bark_detected) {
        return true;
    }
    return false;
}

bool check_arthritis_to_pain(StateMachine* sm) {
    // Condition worsens + limited mobility
    if (sm->sensor_data.range_of_motion < 50.0f &&  // Reduced to 50% ROM
        sm->sensor_data.gait_asymmetry) {
        return true;
    }
    return false;
}

bool check_pain_to_licking(StateMachine* sm) {
    // Excessive licking at injury site
    // This would require additional sensor data for specific location
    time_t duration = get_state_duration(sm);
    if (duration > LICKING_EXCESSIVE_SECONDS) {
        return true;
    }
    return false;
}

bool check_stress_to_licking(StateMachine* sm) {
    // Excessive grooming during stress
    time_t duration = get_state_duration(sm);
    if (duration > LICKING_EXCESSIVE_SECONDS &&
        sm->sensor_data.hrv < 50.0f) {
        return true;
    }
    return false;
}

bool check_critical_conditions(StateMachine* sm) {
    // Check for dehydration/heat stress
    if (sm->sensor_data.temperature > FEVER_THRESHOLD &&
        sm->sensor_data.panting_detected) {
        
        time_t duration = time(NULL) - sm->sensor_data.last_drink_time;
        if (duration > DEHYDRATION_DURATION_MINUTES * 60) {
            return true;
        }
    }
    return false;
}

// ====================
// State Machine Core
// ====================

void initialize_state_machine(StateMachine* sm) {
    sm->current_state = STATE_RESTING_IDLE;
    sm->state_start_time = time(NULL);
    
    // Initialize sub-states
    sm->sub_states.eating_drinking = STATE_EATING_DRINKING_DETECTING;
    sm->sub_states.licking_scratching = STATE_LICKING_SCRATCHING_GROOMING;
    sm->sub_states.dehydration = STATE_DEHYDRATION_DETECTED;
    sm->sub_states.respiration = STATE_RESPIRATION_NORMAL;
    sm->sub_states.temperature = STATE_TEMPERATURE_NORMAL;
    sm->sub_states.hrv = STATE_HRV_STABLE;
    
    // Set configuration
    sm->resting_heart_rate = RESTING_HR_THRESHOLD;
    sm->normal_temp_min = HYPOTHERMIA_THRESHOLD;
    sm->normal_temp_max = FEVER_THRESHOLD;
    sm->idle_to_sleep_minutes = IDLE_TO_SLEEP_MINUTES;
    
    // Initialize persistence
    sm->was_active = false;
    sm->last_hrv = 0.0f;
    sm->last_heart_rate = 0.0f;
	
	/* ================= TEMPERATURE ================= */
	sm->sub_states.temperature = STATE_TEMPERATURE_NORMAL;
	sm->sub_states.temperature_candidate = STATE_TEMPERATURE_NORMAL;
	sm->sub_states.temperature_timer = 0;

	/* ================= RESPIRATION ================= */
	sm->sub_states.respiration = STATE_RESPIRATION_NORMAL;
	sm->sub_states.respiration_candidate = STATE_RESPIRATION_NORMAL;
	sm->sub_states.respiration_timer = 0;

	/* ================= HRV ================= */
	sm->sub_states.hrv = STATE_HRV_STABLE;
	sm->sub_states.hrv_candidate = STATE_HRV_STABLE;
	sm->sub_states.hrv_timer = 0;

	/* ================= DEHYDRATION ================= */
	sm->sub_states.dehydration = STATE_NO_DEHYDRATION;
	sm->sub_states.dehydration_candidate = STATE_NO_DEHYDRATION;
	sm->sub_states.dehydration_timer = 0;

	/* ================= EATING / DRINKING ================= */
	sm->sub_states.eating_drinking = STATE_EATING_DRINKING_IDLE;
	sm->sub_states.eating_drinking_candidate = STATE_EATING_DRINKING_IDLE;
	sm->sub_states.eating_drinking_timer = 0;

	/* ================= LICKING / SCRATCHING ================= */
	sm->sub_states.licking_scratching = STATE_LICKING_SCRATCHING_IDLE;
	sm->sub_states.licking_scratching_candidate = STATE_LICKING_SCRATCHING_IDLE;
	sm->sub_states.licking_scratching_timer = 0;
	
}

void process_state_transitions(StateMachine* sm) {
    StateType new_state = sm->current_state;
   // printf("Acc: %.3f %.3f %.3f Gyro: %.2f %.2f %.2f Step_fre: %.3f Var: %.3f", sm->sensor_data.accel_x, sm->sensor_data.accel_y, 
		//sm->sensor_data.accel_z, sm->sensor_data.gyro_x, sm->sensor_data.gyro_y, sm->sensor_data.gyro_z, sm->sensor_data.step_frequency, 
		//	sm->sensor_data.acc_variance);
#if DEBUG_LOGS
		    printf("Step_fre: %.3f \nVar: %.3f\n", sm->sensor_data.step_frequency, sm->sensor_data.acc_variance);
#endif 
	switch (sm->current_state) {
        case STATE_RESTING_IDLE:
			
			if (check_walking_to_running(sm)) {
		       new_state = STATE_RUNNING_PLAYING;
		    } else if (check_resting_idle_to_walking(sm)) {
                new_state = STATE_WALKING;
            } else if (check_resting_idle_to_sleeping(sm)) {
                new_state = STATE_SLEEPING;
            } else if (check_resting_idle_to_barking(sm)) {
                new_state = STATE_BARKING;
            } else if (check_resting_idle_to_eating_drinking(sm)) {
                new_state = STATE_EATING_DRINKING_DETECTING;
            } else if (check_resting_idle_to_trembling(sm)) {
                new_state = STATE_TREMBLING_SHAKING;
            } else if (check_critical_conditions(sm)) {
                new_state = STATE_DEHYDRATION_DETECTED;
            }
            break;
            
        case STATE_WALKING:
            if (check_walking_to_running(sm)) {
                new_state = STATE_RUNNING_PLAYING;
            } else if (check_walking_to_resting(sm)) {
                new_state = STATE_RESTING_IDLE;
            } else if (check_walking_to_pain_injury(sm)) {
                new_state = STATE_PAIN_INJURY;
            } else if (check_walking_to_arthritis(sm)) {
                new_state = STATE_ARTHRITIS;
            }
            break;
            
        case STATE_RUNNING_PLAYING:
            if (check_running_to_fatigue(sm)) {
                new_state = STATE_FATIGUE_ALERT;
            } else if (check_walking_to_resting(sm)) {
                new_state = STATE_WALKING;
            } else if (sm->sensor_data.bark_detected) {
                new_state = STATE_BARKING;
            } else if (check_critical_conditions(sm)) {
                new_state = STATE_DEHYDRATION_DETECTED;
            }
            break;
            
        case STATE_SLEEPING:
            if (sm->sensor_data.acc_variance > 0.1f) {
                new_state = STATE_RESTING_IDLE;
            } else if (check_sleeping_to_epilepsy(sm)) {
                new_state = STATE_EPILEPSY;
            } else if (check_sleeping_to_trembling(sm)) {
                new_state = STATE_TREMBLING_SHAKING;
            }
            break;
            
            
        case STATE_TREMBLING_SHAKING:
            if (!detect_tremor_pattern(&sm->sensor_data)) {
                new_state = STATE_RESTING_IDLE;
            } else if (check_trembling_to_epilepsy(sm)) {
                new_state = STATE_EPILEPSY;
            } else if (check_trembling_to_stress(sm)) {
                new_state = STATE_STRESS_ANXIETY;
            }
            break;
            
        case STATE_ARTHRITIS:
            if (check_arthritis_to_pain(sm)) {
                new_state = STATE_PAIN_INJURY;
            } else if (sm->sensor_data.range_of_motion > 85.0f) {
                new_state = STATE_RESTING_IDLE;
            }
            break;
            
        case STATE_PAIN_INJURY:
            if (!sm->sensor_data.gait_asymmetry && 
                sm->sensor_data.hrv > sm->last_hrv * 0.9f) {
                new_state = STATE_RESTING_IDLE;
            } else if (check_pain_to_licking(sm)) {
                new_state = STATE_LICKING_SCRATCHING_EXCESSIVE;
            }
            break;
            
        case STATE_STRESS_ANXIETY:
            if (sm->sensor_data.heart_rate <= sm->resting_heart_rate * 1.1f &&
                sm->sensor_data.hrv > 70.0f) {
                new_state = STATE_RESTING_IDLE;
            } else if (check_stress_to_licking(sm)) {
                new_state = STATE_LICKING_SCRATCHING_EXCESSIVE;
            }
            break;
            
        case STATE_FATIGUE_ALERT:
            if (sm->sensor_data.heart_rate <= sm->resting_heart_rate * 1.1f &&
                sm->sensor_data.hrv > sm->last_hrv) {
                new_state = STATE_RESTING_IDLE;
            }
            break;
            
        case STATE_EPILEPSY:
            if (!detect_seizure_pattern(&sm->sensor_data)) {
                new_state = STATE_POST_SEIZURE;
            }
            break;
            
        case STATE_POST_SEIZURE:
            if (sm->sensor_data.heart_rate > sm->resting_heart_rate * 1.3f) {
                new_state = STATE_FATIGUE_ALERT;
            } else {
                new_state = STATE_RESTING_IDLE;
            }
            break;
            
        // Sub-state transitions would be handled separately
        default:
            break;
    }
    
    // Execute state transition if needed
    if (new_state != sm->current_state) {
        exit_state(sm, sm->current_state);
        enter_state(sm, new_state);
        sm->current_state = new_state;
        sm->state_start_time = time(NULL);
		
		printf("Time = %lld \n",(long long)sm->state_start_time);
    }
    
    // Update continuous monitors
    update_monitors(sm);
}

void enter_state(StateMachine* sm, StateType new_state) {
    // Save previous state data
    sm->last_hrv = sm->sensor_data.hrv;
    sm->last_heart_rate = sm->sensor_data.heart_rate;
    
    // State-specific entry actions
    switch (new_state) {
        case STATE_RESTING_IDLE:
            printf("Entering Resting/Idle state\n");
            break;
        case STATE_WALKING:
            printf("Entering Walking state\n");
            sm->sensor_data.last_activity_time = time(NULL);
            break;
        case STATE_RUNNING_PLAYING:
            printf("Entering Running/Playing state\n");
            sm->sensor_data.last_activity_time = time(NULL);
            break;
        case STATE_SLEEPING:
            printf("Entering Sleeping state\n");
            break;
        case STATE_EPILEPSY:
            printf("ALERT: Epileptic seizure detected!\n");
            // Send emergency notification
            break;
        case STATE_DEHYDRATION_DETECTED:
            printf("ALERT: Dehydration/Heat stress detected!\n");
            break;
        default:
            break;
    }
}

void exit_state(StateMachine* sm, StateType old_state) {
    // State-specific exit actions
    switch (old_state) {
        case STATE_EATING_DRINKING_ACTIVE:
            sm->sensor_data.last_drink_time = time(NULL);
            break;
        default:
            break;
    }
}

// ====================
// Monitor Functions
// ====================
void update_monitors(StateMachine* sm)
{
    uint32_t now = GET_TIME_MS();
    StateType new_state;

    /* ================= TEMPERATURE ================= */
    if (sm->sensor_data.temperature > FEVER_THRESHOLD)
        new_state = STATE_TEMPERATURE_FEVER;
    else if (sm->sensor_data.temperature < HYPOTHERMIA_THRESHOLD)
        new_state = STATE_TEMPERATURE_HYPOTHERMIA;
    else
        new_state = STATE_TEMPERATURE_NORMAL;

    if (new_state != sm->sub_states.temperature) {
        if (new_state != sm->sub_states.temperature_candidate) {
            sm->sub_states.temperature_candidate = new_state;
            sm->sub_states.temperature_timer = now;
        }
        else if (now - sm->sub_states.temperature_timer >= TEMP_CONFIRM_MS) {
            sm->sub_states.temperature = new_state;
        }
    }

    /* ================= RESPIRATION ================= */
    if (sm->sensor_data.respiration_rate > RESPIRATION_NORMAL_MAX)
        new_state = STATE_RESPIRATION_ELEVATED;
	else if( sm->sensor_data.respiration_rate < RESPIRATION_NORMAL_MIN) 
		new_state = STATE_RESPIRATION_LOW;
	else 
        new_state = STATE_RESPIRATION_NORMAL;

    if (new_state != sm->sub_states.respiration) {
        if (new_state != sm->sub_states.respiration_candidate) {
            sm->sub_states.respiration_candidate = new_state;
            sm->sub_states.respiration_timer = now;
        }
        else if (now - sm->sub_states.respiration_timer >= RESP_CONFIRM_MS) {
            sm->sub_states.respiration = new_state;
        }
    }

    /* ================= HRV ================= */
	if(sm->sensor_data.hrv == 0)
		new_state = STATE_HRV_UNKNOW;
    else if (sm->sensor_data.hrv < HRV_LOW && sm->sensor_data.hrv > 0)
        new_state = STATE_HRV_STRESSED;
	else
        new_state = STATE_HRV_STABLE;

    if (new_state != sm->sub_states.hrv) {
        if (new_state != sm->sub_states.hrv_candidate) {
            sm->sub_states.hrv_candidate = new_state;
            sm->sub_states.hrv_timer = now;
        }
        else if (now - sm->sub_states.hrv_timer >= HRV_CONFIRM_MS) {
            sm->sub_states.hrv = new_state;
        }
    }
	
	/* ================= HR ================= */
	if(sm->sensor_data.heart_rate == 0)
		new_state = STATE_HR_UNKNOW;
	else if (sm->sensor_data.heart_rate < HIGH_HR_THRESHOLD && sm->sensor_data.heart_rate> LOW_HR_THRESHOLD)
	    new_state = STATE_HR_STABLE;
	else if (sm->sensor_data.heart_rate < LOW_HR_THRESHOLD && sm->sensor_data.heart_rate > 0)
	    new_state = STATE_HR_LOW;
	else
		new_state = STATE_HR_HIGH;
	 

	if (new_state != sm->sub_states.hr) {
	    if (new_state != sm->sub_states.hr_candidate) {
	        sm->sub_states.hr_candidate = new_state;
	        sm->sub_states.hr_timer = now;
	    }
	    else if (now - sm->sub_states.hr_timer >= HR_CONFIRM_MS) {
	        sm->sub_states.hr = new_state;
	    }
	}
	
}
// ====================
// Helper Functions
// ====================

bool detect_step_pattern(SensorData* data) {
    // Detect rhythmic patterns in IMU data
    // This would use FFT or peak detection in a real implementation
    return (data->step_frequency > 0.5f);
}

bool detect_tremor_pattern(SensorData* data) {
    // Check for 5-12Hz vibration patterns
    return (data->step_frequency >= TREMOR_FREQ_MIN &&
            data->step_frequency <= TREMOR_FREQ_MAX);
}

bool detect_seizure_pattern(SensorData* data) {
    // Detect violent jerking motions
    return (data->acc_variance > 5.0f);  // High variance threshold
}

bool detect_head_down_posture(SensorData* data) {
    // Detect head-down posture (negative Z acceleration)
    return (data->accel_z < -0.5f);
}

time_t get_state_duration(StateMachine* sm) {
    return time(NULL) - sm->state_start_time;
}

bool is_hrv_stable(SensorData* data, float threshold) {
    // Check if HRV is within stable range
    return (data->hrv > threshold);
}

void show_state(StateMachine* sm) {
    // State-specific entry actions
    switch (sm->current_state) {
        case STATE_RESTING_IDLE:
            printf("Resting\n");
            break;
        case STATE_WALKING:
            printf("Walking\n");
            break;
        case STATE_RUNNING_PLAYING:
            printf("Running/Playing\n");
            break;
        case STATE_SLEEPING:
            printf("Sleeping\n");
            break;
        case STATE_BARKING:
            printf("Barking\n");
            break;
        case STATE_TREMBLING_SHAKING:
            printf("Trembling\n");
            break;
        case STATE_EPILEPSY:
            printf("ALERT: Epileptic seizure detected!\n");
            break;
        case STATE_POST_SEIZURE:
            printf("ALERT: Post seizure detected!\n");
            break;
        case STATE_ARTHRITIS:
            printf("ALERT: Arthritis detected!\n");
            break;
        case STATE_PAIN_INJURY:
            printf("ALERT: Pain Injury detected!\n");
            break;
        case STATE_FATIGUE_ALERT:
            printf("ALERT: Fatigue detected!\n");
            break;
        case STATE_STRESS_ANXIETY:
            printf("ALERT: Stress detected!\n");
            break;
        case STATE_EATING_DRINKING_DETECTING:
            printf("ALERT: Eating/Drinking detected!\n");
            break;
        case STATE_EATING_DRINKING_ACTIVE:
            printf("ALERT: Eating/Drinking Active detected!\n");
            break;
        case STATE_EATING_DRINKING_COMPLETE:
            printf("ALERT: Eating/Drinking complete detected!\n");
            break;
        case STATE_LICKING_SCRATCHING_GROOMING:
            printf("ALERT: Licking/Scratching Grooming detected!\n");
            break;
        case STATE_LICKING_SCRATCHING_EXCESSIVE:
            printf("ALERT: Licking/Scratching Excessive detected!\n");
            break;
        case STATE_LICKING_SCRATCHING_MEDICAL_ALERT:
            printf("ALERT: Licking/Scratching Medical detected!\n");
            break;
        case STATE_DEHYDRATION_DETECTED:
            printf("ALERT: Dehydration detected!\n");
            break;
        case STATE_DEHYDRATION_SEVERE:
            printf("ALERT: Dehydration Severe detected!\n");
            break;
        case STATE_DEHYDRATION_CRITICAL:
            printf("ALERT: Dehydration Critical detected!\n");
            break;
        case STATE_RESPIRATION_NORMAL:
            printf("ALERT: Respiration Normal detected!\n");
            break;
        case STATE_RESPIRATION_ELEVATED:
            printf("ALERT: Respiration Elevated detected!\n");
            break;
        case STATE_TEMPERATURE_NORMAL:
            printf("ALERT: Temperature Normal detected!\n");
            break;
        case STATE_TEMPERATURE_FEVER:
            printf("ALERT: Temperature fever detected!\n");
            break;
        case STATE_TEMPERATURE_HYPOTHERMIA:
            printf("ALERT: Temperature Hypothermia detected!\n");
            break;
        case STATE_HRV_STABLE:
            printf("ALERT: HRV Stable detected!\n");
            break;
        case STATE_HRV_STRESSED:
            printf("ALERT: HRV stressed detected!\n");
            break;
        case STATE_UNKNOWN:
            printf("UNKNOWN!\n");
            break;
        default:
            break;
    }
}


//---------------------------------

sensor_data_t sensor_data;
alert_flags_t alert_flags;

uint32_t evaluate_alerts(sensor_data_t *d)
{
    uint32_t alerts = ALERT_NONE;

    /* Dehydration / Heat Stress */
    if (d->heart_rate > HR_HIGH &&
        d->body_temp > BODY_TEMP_HIGH &&
        d->activity_level < ACTIVITY_LOW &&
        d->ambient_temp > AMBIENT_TEMP_HIGH)
    {
        alerts |= ALERT_DEHYDRATION;
    }

    /* Arthritis / Hip Dysplasia */
    if (d->activity_level < ACTIVITY_LOW &&
        d->gait_variance > GAIT_IRREGULAR)
    {
        alerts |= ALERT_ARTHRITIS;
    }

    /* Fever / Infection */
    if (d->body_temp > BODY_TEMP_HIGH &&
        d->activity_level < ACTIVITY_LOW)
    {
        alerts |= ALERT_FEVER;
    }

    /* Heart Distress */
    if (d->heart_rate > HR_REST_HIGH &&
        d->activity_level < ACTIVITY_LOW)
    {
        alerts |= ALERT_HEART_DISTRESS;
    }

    /* Anxiety / Stress */
    if (d->hrv < HRV_LOW &&
        d->night_motion > NIGHT_MOTION_HIGH)
    {
        alerts |= ALERT_ANXIETY;
    }

    /* Respiratory Distress */
    if (d->breathing_rate > BREATH_RATE_HIGH &&
        d->activity_level < ACTIVITY_LOW)
    {
        alerts |= ALERT_RESPIRATORY;
    }

    /* Pain / Injury */
    if (d->heart_rate > HR_HIGH &&
        d->activity_level < ACTIVITY_LOW)
    {
        alerts |= ALERT_PAIN;
    }

    /* Overheating */
    if (d->body_temp > BODY_TEMP_HIGH &&
        d->ambient_temp > AMBIENT_TEMP_HIGH &&
        d->activity_level < ACTIVITY_LOW)
    {
        alerts |= ALERT_OVERHEAT;
    }

    /* Lethargy */
    if (d->activity_level < ACTIVITY_LOW &&
        d->hrv < HRV_LOW)
    {
        alerts |= ALERT_LETHARGY;
    }

    /* Sleep Disruption */
    if (d->night_motion > NIGHT_MOTION_HIGH)
    {
        alerts |= ALERT_SLEEP_DISTURBANCE;
    }

    return alerts;
}


void process_sensor_stage(sensor_data_t *data)
{
    uint32_t alerts = evaluate_alerts(data);

    if (alerts & ALERT_DEHYDRATION)
        printf("Alert: Dehydration / Heat Stress\n");

    if (alerts & ALERT_ARTHRITIS)
        printf("Alert: Early Arthritis / Hip Dysplasia\n");

    if (alerts & ALERT_FEVER)
        printf("Alert: Fever / Infection\n");

    if (alerts & ALERT_HEART_DISTRESS)
        printf("Alert: Heart Distress\n");

    if (alerts & ALERT_ANXIETY)
        printf("Alert: Anxiety / Stress\n");

    if (alerts & ALERT_RESPIRATORY)
        printf("Alert: Respiratory Distress\n");

    if (alerts & ALERT_PAIN)
        printf("Alert: Pain / Injury\n");

    if (alerts & ALERT_OVERHEAT)
        printf("Alert: Overheating\n");

    if (alerts & ALERT_LETHARGY)
        printf("Alert: Lethargy / Fatigue\n");

    if (alerts & ALERT_SLEEP_DISTURBANCE)
        printf("Alert: Sleep Disturbance\n");
}

void evaluate_alerts_function(void){
	evaluate_alerts(&sensor_data);
	process_sensor_stage(&sensor_data);
}



//-------------------------

