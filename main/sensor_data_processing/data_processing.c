#include <stdio.h>
#include "data_processing.h"
#include "signal_list.h"
#include <math.h>

extern s_all_raw_signals_t raw_signals;

// Function definition for raw data processing and passing values to feature processing
float cal_accel_magnitude(s_all_raw_signals_t* raw_signals)
{
    return sqrt(pow(raw_signals->accel_x, 2) +
                pow(raw_signals->accel_y, 2) +
                pow(raw_signals->accel_z, 2));
}

float cal_gyro_magnitude(s_all_raw_signals_t* raw_signals)
{
    return sqrt(pow(raw_signals->gyro_x, 2) +
                pow(raw_signals->gyro_y, 2) +
                pow(raw_signals->gyro_z, 2));
}

float cal_mag_magnitude(s_all_raw_signals_t* raw_signals)
{
    return sqrt(pow(raw_signals->mag_x, 2) +
                pow(raw_signals->mag_y, 2) +
                pow(raw_signals->mag_z, 2));
}

//other signal processing functionalities to be added(e.g., sound amplitude, frequency, heart rate, respiratory rate)
int cal_sound_amplitude(s_all_raw_signals_t* raw_signals)
{
    return 0; // Placeholder
}

int cal_sound_frequency(s_all_raw_signals_t* raw_signals)
{
    return 0; // Placeholder
}

int cal_heart_rate(s_all_raw_signals_t* raw_signals)
{
    return 0; // Placeholder
}

int cal_respiratory_rate(s_all_raw_signals_t* raw_signals)
{
    return 0; // Placeholder
}


int set_processed_signals(s_all_processed_signals_t* processed_signals, s_all_raw_signals_t* raw_signals)
{
    // Calculate magnitudes
    processed_signals->accel_mag = cal_accel_magnitude(raw_signals);
    printf("ACC: %.3f %.3f %.3f\n", fabs(raw_signals->accel_x), fabs(raw_signals->accel_y), fabs(raw_signals->accel_z));
    printf("Acceleration Mag: %.3f\n", processed_signals->accel_mag);
    processed_signals->gyro_mag = cal_gyro_magnitude(raw_signals);
    printf("GYR: %.2f %.2f %.2f\n", raw_signals->gyro_x, raw_signals->gyro_y, raw_signals->gyro_z);
    printf("Gyroscope Mag: %.2f\n", processed_signals->gyro_mag);
    // processed_signals->mag_mag = cal_mag_magnitude(raw_signals);

    // Calculate heart rate
    processed_signals->heart_rate = raw_signals->heart_rate;//cal_heart_rate(raw_signals);

    // // Calculate respiratory rate
    // processed_signals->respiratory_rate = cal_respiratory_rate(raw_signals);

    // // Calculate sound amplitude
    // processed_signals->sound_amp = cal_sound_amplitude(raw_signals);

    // // Calculate sound frequency
    // processed_signals->sound_freq = cal_sound_frequency(raw_signals);

    // // Set temperature
    // processed_signals->temp = raw_signals->temp;

    return 0; // Indicate success
}