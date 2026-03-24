/*
 * feature_processing.c
 * Application layer – dog behaviour state machine.
 * Logic unchanged from original; includes updated to new paths.
 */

#include "feature_processing.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_timer.h"

StateMachine sm;

/* ── State transition checks ────────────────────────────────── */

bool check_resting_idle_to_walking(StateMachine *sm) {
    return (sm->sensor_data.step_frequency >= STEP_FREQ_WALKING_MIN &&
            sm->sensor_data.step_frequency <= STEP_FREQ_WALKING_MAX &&
            sm->sensor_data.acc_variance > 1.0f);
}

bool check_walking_to_running(StateMachine *sm) {
    return (sm->sensor_data.step_frequency > STEP_FREQ_RUNNING_MIN &&
            sm->sensor_data.acc_variance > 1.0f);
}

bool check_walking_to_resting(StateMachine *sm) {
    return (get_state_duration(sm) >= MOTION_STOP_SECONDS &&
            sm->sensor_data.acc_variance < 0.1f);
}

bool check_resting_idle_to_sleeping(StateMachine *sm) {
    time_t dur = get_state_duration(sm);
    if (dur < IDLE_TO_SLEEP_MINUTES * 60) return false;
    if (sm->last_heart_rate <= 0) return false;
    float hr_drop = ((sm->last_heart_rate - sm->sensor_data.heart_rate) /
                     (float)sm->last_heart_rate) * 100.0f;
    return (hr_drop >= 10.0f && sm->sensor_data.hrv > sm->last_hrv);
}

bool check_resting_idle_to_barking(StateMachine *sm) {
    (void)sm;
    return false;   /* audio/bark detection removed */
}

bool check_resting_idle_to_eating_drinking(StateMachine *sm) {
    (void)sm;
    return false;   /* crunch/lap sound detection removed */
}

bool check_resting_idle_to_trembling(StateMachine *sm) {
    return (detect_tremor_pattern(&sm->sensor_data) &&
            (sm->current_state == STATE_RESTING_IDLE ||
             sm->current_state == STATE_SLEEPING));
}

bool check_running_to_fatigue(StateMachine *sm) {
    return (get_state_duration(sm) > 600 &&
            sm->sensor_data.heart_rate > RESTING_HR_THRESHOLD * 1.5f &&
            sm->sensor_data.acc_variance < 0.5f);
}

bool check_sleeping_to_epilepsy(StateMachine *sm) {
    return (detect_seizure_pattern(&sm->sensor_data) &&
            sm->sensor_data.heart_rate > HIGH_HR_THRESHOLD);
}

bool check_sleeping_to_trembling(StateMachine *sm) {
    return (detect_tremor_pattern(&sm->sensor_data) &&
            sm->sensor_data.heart_rate > sm->last_heart_rate * 1.2f);
}

bool check_trembling_to_epilepsy(StateMachine *sm) {
    return detect_seizure_pattern(&sm->sensor_data);
}

bool check_trembling_to_stress(StateMachine *sm) {
    return (sm->sensor_data.hrv < 50.0f &&
            sm->sensor_data.heart_rate > RESTING_HR_THRESHOLD * 1.2f);
}

bool check_barking_to_stress(StateMachine *sm) {
    return (sm->sensor_data.heart_rate > RESTING_HR_THRESHOLD * 1.3f &&
            sm->sensor_data.hrv < 50.0f);
}
bool check_arthritis_to_pain(StateMachine *sm) {
    return (sm->sensor_data.range_of_motion < 50.0f &&
            sm->sensor_data.gait_asymmetry);
}

bool check_walking_to_pain_injury(StateMachine *sm) {
    return (sm->sensor_data.gait_asymmetry &&
            sm->sensor_data.hrv < sm->last_hrv * 0.8f);
}

bool check_walking_to_arthritis(StateMachine *sm) {
    if (sm->sensor_data.range_of_motion >= 70.0f) return false;
    time_t idle = time(NULL) - sm->sensor_data.last_activity_time;
    return (idle > 300);
}

bool check_stress_to_licking(StateMachine *sm) {
    return (get_state_duration(sm) > LICKING_EXCESSIVE_SECONDS &&
            sm->sensor_data.hrv < 50.0f);
}

bool check_pain_to_licking(StateMachine *sm) {
    return (get_state_duration(sm) > LICKING_EXCESSIVE_SECONDS);
}

bool check_critical_conditions(StateMachine *sm) {
    if (sm->sensor_data.temperature <= FEVER_THRESHOLD) return false;
    time_t no_drink = time(NULL) - sm->sensor_data.last_drink_time;
    return (no_drink > DEHYDRATION_DURATION_MINUTES * 60);
}

/* ── State enter / exit ──────────────────────────────────────── */
void enter_state(StateMachine *sm, StateType new_state) {
    sm->last_hrv         = sm->sensor_data.hrv;
    sm->last_heart_rate  = sm->sensor_data.heart_rate;
    switch (new_state) {
        case STATE_WALKING:
        case STATE_RUNNING_PLAYING:
            sm->sensor_data.last_activity_time = time(NULL);
            break;
        case STATE_EPILEPSY:
            printf("[ALERT] Epileptic seizure detected!\n");
            break;
        case STATE_DEHYDRATION_DETECTED:
            printf("[ALERT] Dehydration detected!\n");
            break;
        default: break;
    }
}

void exit_state(StateMachine *sm, StateType old_state) {
    if (old_state == STATE_EATING_DRINKING_ACTIVE)
        sm->sensor_data.last_drink_time = time(NULL);
}

/* ── Main transition loop ────────────────────────────────────── */
void process_state_transitions(StateMachine *sm) {
    StateType next = sm->current_state;

    switch (sm->current_state) {
        case STATE_RESTING_IDLE:
            if      (check_walking_to_running(sm))               next = STATE_RUNNING_PLAYING;
            else if (check_resting_idle_to_walking(sm))          next = STATE_WALKING;
            else if (check_resting_idle_to_sleeping(sm))         next = STATE_SLEEPING;
            else if (check_resting_idle_to_trembling(sm))        next = STATE_TREMBLING_SHAKING;
            break;

        case STATE_WALKING:
            if      (check_walking_to_running(sm))        next = STATE_RUNNING_PLAYING;
            else if (check_walking_to_resting(sm))        next = STATE_RESTING_IDLE;
            else if (check_walking_to_pain_injury(sm))    next = STATE_PAIN_INJURY;
            else if (check_walking_to_arthritis(sm))      next = STATE_ARTHRITIS;
            break;

        case STATE_BARKING:
            /* Audio removed — always exit back to resting */
            next = STATE_RESTING_IDLE;
            break;

        case STATE_RUNNING_PLAYING:
            if      (check_running_to_fatigue(sm))  next = STATE_FATIGUE_ALERT;
            else if (check_walking_to_resting(sm))  next = STATE_WALKING;
            break;

        case STATE_SLEEPING:
            if      (sm->sensor_data.acc_variance > 0.1f)  next = STATE_RESTING_IDLE;
            else if (check_sleeping_to_epilepsy(sm))        next = STATE_EPILEPSY;
            else if (check_sleeping_to_trembling(sm))       next = STATE_TREMBLING_SHAKING;
            break;

        case STATE_TREMBLING_SHAKING:
            if      (!detect_tremor_pattern(&sm->sensor_data)) next = STATE_RESTING_IDLE;
            else if (check_trembling_to_epilepsy(sm))          next = STATE_EPILEPSY;
            else if (check_trembling_to_stress(sm))            next = STATE_STRESS_ANXIETY;
            break;

        case STATE_EPILEPSY:
            if (!detect_seizure_pattern(&sm->sensor_data)) next = STATE_POST_SEIZURE;
            break;

        case STATE_POST_SEIZURE:
            next = (sm->sensor_data.heart_rate > RESTING_HR_THRESHOLD * 1.3f)
                   ? STATE_FATIGUE_ALERT : STATE_RESTING_IDLE;
            break;

        case STATE_ARTHRITIS:
            if      (check_arthritis_to_pain(sm))            next = STATE_PAIN_INJURY;
            else if (sm->sensor_data.range_of_motion > 85.f) next = STATE_RESTING_IDLE;
            break;

        case STATE_PAIN_INJURY:
            if      (!sm->sensor_data.gait_asymmetry &&
                     sm->sensor_data.hrv > sm->last_hrv * 0.9f) next = STATE_RESTING_IDLE;
            else if (check_pain_to_licking(sm))                  next = STATE_LICKING_SCRATCHING_EXCESSIVE;
            break;

        case STATE_STRESS_ANXIETY:
            if      (sm->sensor_data.heart_rate <= sm->resting_heart_rate * 1.1f &&
                     sm->sensor_data.hrv > 70.0f) next = STATE_RESTING_IDLE;
            else if (check_stress_to_licking(sm)) next = STATE_LICKING_SCRATCHING_EXCESSIVE;
            break;

        case STATE_FATIGUE_ALERT:
            if (sm->sensor_data.heart_rate <= sm->resting_heart_rate * 1.1f &&
                sm->sensor_data.hrv > sm->last_hrv) next = STATE_RESTING_IDLE;
            break;

        default: break;
    }

    if (next != sm->current_state) {
        exit_state(sm, sm->current_state);
        enter_state(sm, next);
        sm->current_state    = next;
        sm->state_start_time = time(NULL);
    }

    update_monitors(sm);
}

/* ── Monitor sub-states ──────────────────────────────────────── */
void update_monitors(StateMachine *sm)
{
    uint32_t now = GET_TIME_MS();
    StateType ns;

#define CONFIRM(field, timer, confirm_ms) \
    if (ns != sm->sub_states.field) { \
        if (ns != sm->sub_states.field##_candidate) { \
            sm->sub_states.field##_candidate = ns; \
            sm->sub_states.timer = now; \
        } else if (now - sm->sub_states.timer >= (confirm_ms)) { \
            sm->sub_states.field = ns; \
        } \
    }

    /* Temperature */
    ns = (sm->sensor_data.temperature > FEVER_THRESHOLD)     ? STATE_TEMPERATURE_FEVER
       : (sm->sensor_data.temperature < HYPOTHERMIA_THRESHOLD) ? STATE_TEMPERATURE_HYPOTHERMIA
       : STATE_TEMPERATURE_NORMAL;
    CONFIRM(temperature, temperature_timer, TEMP_CONFIRM_MS)

    /* Respiration */
    ns = (sm->sensor_data.respiration_rate > RESPIRATION_NORMAL_MAX) ? STATE_RESPIRATION_ELEVATED
       : (sm->sensor_data.respiration_rate < RESPIRATION_NORMAL_MIN) ? STATE_RESPIRATION_LOW
       : STATE_RESPIRATION_NORMAL;
    CONFIRM(respiration, respiration_timer, RESP_CONFIRM_MS)

    /* HRV */
    ns = (sm->sensor_data.hrv == 0)                          ? STATE_HRV_UNKNOW
       : (sm->sensor_data.hrv < HRV_LOW)                     ? STATE_HRV_STRESSED
       : STATE_HRV_STABLE;
    CONFIRM(hrv, hrv_timer, HRV_CONFIRM_MS)

    /* HR */
    ns = (sm->sensor_data.heart_rate == 0)                   ? STATE_HR_UNKNOW
       : (sm->sensor_data.heart_rate > HIGH_HR_THRESHOLD)    ? STATE_HR_HIGH
       : (sm->sensor_data.heart_rate < LOW_HR_THRESHOLD)     ? STATE_HR_LOW
       : STATE_HR_STABLE;
    CONFIRM(hr, hr_timer, HR_CONFIRM_MS)

    /* Dehydration — sub-state only, never overwrites main state machine.
     * Resets to NO_DEHYDRATION whenever last_drink_time is updated. */
    {
        time_t no_drink = time(NULL) - sm->sensor_data.last_drink_time;
        ns = (no_drink > (DEHYDRATION_DURATION_MINUTES * 60 * 2))
                ? STATE_DEHYDRATION_CRITICAL
           : (no_drink > (DEHYDRATION_DURATION_MINUTES * 60))
                ? STATE_DEHYDRATION_DETECTED
           : STATE_NO_DEHYDRATION;
        CONFIRM(dehydration, dehydration_timer, DEHYD_CONFIRM_MS)
    }

#undef CONFIRM
}

/* ── Helpers ────────────────────────────────────────────────── */
bool detect_tremor_pattern(SensorData *d) {
    return (d->step_frequency >= TREMOR_FREQ_MIN &&
            d->step_frequency <= TREMOR_FREQ_MAX);
}

bool detect_seizure_pattern(SensorData *d) {
    return (d->acc_variance > 5.0f);
}

bool detect_head_down_posture(SensorData *d) {
    return (d->accel_z < -0.5f);
}

time_t get_state_duration(StateMachine *sm) {
    return time(NULL) - sm->state_start_time;
}

/* ── Init ───────────────────────────────────────────────────── */
void initialize_state_machine(StateMachine *sm) {
    memset(sm, 0, sizeof(StateMachine));
    sm->current_state          = STATE_RESTING_IDLE;
    sm->state_start_time       = time(NULL);
    sm->resting_heart_rate     = RESTING_HR_THRESHOLD;
    sm->normal_temp_min        = HYPOTHERMIA_THRESHOLD;
    sm->normal_temp_max        = FEVER_THRESHOLD;
    sm->idle_to_sleep_minutes  = IDLE_TO_SLEEP_MINUTES;

    /* CRITICAL: initialise to NOW so check_critical_conditions() does not
     * immediately fire (last_drink_time=0 means "last drank Jan 1 1970") */
    sm->sensor_data.last_drink_time    = time(NULL);
    sm->sensor_data.last_activity_time = time(NULL);

    sm->sub_states.eating_drinking           = STATE_EATING_DRINKING_IDLE;
    sm->sub_states.eating_drinking_candidate = STATE_EATING_DRINKING_IDLE;
    sm->sub_states.licking_scratching           = STATE_LICKING_SCRATCHING_IDLE;
    sm->sub_states.licking_scratching_candidate = STATE_LICKING_SCRATCHING_IDLE;
    sm->sub_states.dehydration           = STATE_NO_DEHYDRATION;
    sm->sub_states.dehydration_candidate = STATE_NO_DEHYDRATION;
    sm->sub_states.respiration           = STATE_RESPIRATION_NORMAL;
    sm->sub_states.respiration_candidate = STATE_RESPIRATION_NORMAL;
    sm->sub_states.temperature           = STATE_TEMPERATURE_NORMAL;
    sm->sub_states.temperature_candidate = STATE_TEMPERATURE_NORMAL;
    sm->sub_states.hrv           = STATE_HRV_STABLE;
    sm->sub_states.hrv_candidate = STATE_HRV_STABLE;
    sm->sub_states.hr            = STATE_HR_UNKNOW;
    sm->sub_states.hr_candidate  = STATE_HR_UNKNOW;
}

/* ── Debug print ────────────────────────────────────────────── */
void show_state(StateMachine *sm) {
    static const char *names[] = {
        "RESTING_IDLE","WALKING","RUNNING_PLAYING","SLEEPING",
        "BARKING","TREMBLING","EPILEPSY","POST_SEIZURE",
        "ARTHRITIS","PAIN_INJURY","FATIGUE","STRESS_ANXIETY",
        "EAT_IDLE","EAT_DETECTING","EAT_ACTIVE","EAT_COMPLETE",
        "LICK_IDLE","LICK_GROOMING","LICK_EXCESSIVE","LICK_MEDICAL",
        "NO_DEHYDRATION","DEHYDRATION","DEHYD_SEVERE","DEHYD_CRITICAL",
        "RESP_NORMAL","RESP_LOW","RESP_ELEVATED",
        "TEMP_NORMAL","TEMP_FEVER","TEMP_HYPOTHERMIA",
        "HRV_UNKNOWN","HRV_STABLE","HRV_STRESSED",
        "HR_UNKNOWN","HR_STABLE","HR_HIGH","HR_LOW",
        "UNKNOWN"
    };
    int idx = (int)sm->current_state;
    int max = (int)(sizeof(names)/sizeof(names[0]));
    printf("STATE: %s\n", (idx >= 0 && idx < max) ? names[idx] : "?");
}
