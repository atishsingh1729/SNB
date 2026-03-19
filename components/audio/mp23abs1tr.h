/*
 * mp23abs1tr.h
 *
 *  Created on: Dec 24, 2025
 *      Author: Dheeraj.Jain
 */

#ifndef COMPONENTS_AUDIO_MP23ABS1TR_H_
#define COMPONENTS_AUDIO_MP23ABS1TR_H_

#include <stdint.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BARK_THRESHOLD_DB  65.0f   // Bark detection threshold
// Add these constants at the top
#define CRUNCH_FREQUENCY_MIN    500   // Hz - lower frequency for crunching
#define CRUNCH_FREQUENCY_MAX    1500  // Hz
#define LAP_FREQUENCY_MIN       100   // Hz - lapping sound frequency
#define LAP_FREQUENCY_MAX       800   // Hz
#define DRINKING_DURATION_SEC   5     // Minimum duration to consider as drinking
#define EATING_DURATION_SEC     10    // Minimum duration to consider as eating

#define ADC_CHANNEL_AUDIO	ADC_CHANNEL_6    	// GPIO7

#define SAMPLE_COUNT         32
#define SAMPLE_DELAY_US      50   // ~20 kHz

float mp23abs1_read_rms_mv(void);

#endif /* COMPONENTS_AUDIO_MP23ABS1TR_H_ */
