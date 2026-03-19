#ifndef SIGNAL_LIST_H
#define SIGNAL_LIST_H

#include <stdint.h>

//structure of raw signals
typedef struct raw_signals {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float mag_x;
    float mag_y;
    float mag_z;
    uint8_t temp;
    uint8_t heart_rate;
    uint16_t sound;
    //gps data not yet included
} s_all_raw_signals_t; 
extern s_all_raw_signals_t raw_signals;

//structure of processed signals
typedef struct processed_signals {
    uint8_t heart_rate;
    uint8_t respiratory_rate;
    uint8_t temp;
    float accel_mag;
    float gyro_mag;
    float mag_mag;
    uint16_t sound_amp;
    uint16_t sound_freq;
    //gps data not yet included
} s_all_processed_signals_t;
extern s_all_processed_signals_t processed_signals;

#endif // SIGNAL_LIST_H