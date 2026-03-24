/*
 * feature_processing_feed.c
 *
 * Thin glue that copies sensor_raw_t + sensor_processed_t values
 * into StateMachine.sensor_data so the existing state-transition
 * logic requires zero changes.
 *
 * The full state-machine code (process_state_transitions, update_monitors,
 * initialize_state_machine, etc.) lives in feature_processing.c which is
 * unchanged except for updated #include paths.
 */

#include "feature_processing.h"

void feature_processing_feed(StateMachine             *sm,
                              const sensor_raw_t       *raw,
                              const sensor_processed_t *proc)
{
    SensorData *s = &sm->sensor_data;

    /* IMU */
    s->accel_x      = raw->accel_x;
    s->accel_y      = raw->accel_y;
    s->accel_z      = raw->accel_z;
    s->gyro_x       = raw->gyro_x;
    s->gyro_y       = raw->gyro_y;
    s->gyro_z       = raw->gyro_z;
    s->lin_accel_x  = raw->lin_accel_x;
    s->lin_accel_y  = raw->lin_accel_y;
    s->lin_accel_z  = raw->lin_accel_z;
    s->magneto_x    = raw->mag_x;
    s->magneto_y    = raw->mag_y;
    s->magneto_z    = raw->mag_z;
    s->roll         = raw->roll;
    s->pitch        = raw->pitch;
    s->yaw          = raw->yaw;
    s->total_steps  = raw->total_steps;
    s->step_frequency = raw->step_frequency;

    /* Vitals */
    s->heart_rate       = raw->heart_rate;
    s->hrv              = raw->hrv;
    s->rawIR            = raw->raw_ir;
    s->temperature      = raw->ambient_temp;

    /* From sensor processing */
    s->acc_variance        = raw->acc_variance;      /* from BNO085 circular buffer */
    s->range_of_motion     = proc->range_of_motion;
    s->respiration_rate    = proc->respiration_rate;
    s->calorie_expenditure = proc->calorie_expenditure;
    s->gait_asymmetry      = proc->gait_asymmetry;
}
