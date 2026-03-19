#ifndef DATA_PROCESSING_H
#define DATA_PROCESSING_H

#include "signal_list.h"

//should I make a single function that takes in raw signals structure and calculates magnitudes?
float cal_accel_magnitude(s_all_raw_signals_t* raw_signals);
float cal_gyro_magnitude(s_all_raw_signals_t* raw_signals);
float cal_mag_magnitude(s_all_raw_signals_t* raw_signals);
int cal_sound_amplitude(s_all_raw_signals_t* raw_signals);
int cal_sound_frequency(s_all_raw_signals_t* raw_signals);
int cal_heart_rate(s_all_raw_signals_t* raw_signals);
int cal_respiratory_rate(s_all_raw_signals_t* raw_signals);

 //set data to processed signals structure
int set_processed_signals(s_all_processed_signals_t* processed_signals, s_all_raw_signals_t* raw_signals);

#endif // DATA_PROCESSING_H