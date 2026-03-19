/*
 * hr_fft.h
 *
 *  Created on: Jan 29, 2026
 *      Author: Sanjay.Chauhan
 */

#ifndef COMPONENTS_IMU_HR_FFT_H_
#define COMPONENTS_IMU_HR_FFT_H_


#pragma once
#include <stdint.h>

void hr_fft_init(void);
void hr_fft_push_sample(float ir_ac);
uint32_t hr_fft_get_bpm(void);



#endif /* COMPONENTS_IMU_HR_FFT_H_ */
