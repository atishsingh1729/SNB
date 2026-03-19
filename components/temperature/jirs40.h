/*
 * jirs40.h
 *
 *  Created on: Dec 24, 2025
 *      Author: Dheeraj.Jain
 */

#ifndef COMPONENTS_TEMPERATURE_JIRS40_H_
#define COMPONENTS_TEMPERATURE_JIRS40_H_

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* ------------ ADC CONFIG ------------ */
#define ADC_UNIT_USED        ADC_UNIT_1
#define ADC_ATTEN_USED       ADC_ATTEN_DB_11
#define ADC_BITWIDTH_USED    ADC_BITWIDTH_DEFAULT

#define ADC_CH_BODY          ADC_CHANNEL_0   // GPIO1 (ADC1_CH0)
#define ADC_CH_AMB           ADC_CHANNEL_6   // GPIO7 (ADC1_CH6)

/* ------------ NTC CONFIG ------------ */
#define ADC_VREF             3.289f
#define ADC_MAX              4095.0f

#define NTC_R0               10000.0f   // 10k NTC
#define NTC_T0               298.15f    // 25°C in Kelvin
#define NTC_BETA             3950.0f    // Typical 10k NTC

#define SERIES_RESISTOR      10000.0f   // 10k divider resistor

/* ------------ OPTIONAL CALIBRATION ------------ */
#define NTC_CAL_SLOPE        1.216f
#define NTC_CAL_OFFSET_C   -5.37f

#define ADC_BATT_GAIN 		1.033f


#define  ADC_CH_BATT	ADC_CHANNEL_5

void adc_init(void);
float adc_read(adc_channel_t channel, adc_cali_handle_t cali_handle);
float ntc_voltage_to_resistance(float vout);
float ntc_resistance_to_temp(float r_ntc);
float ntc_apply_calibration(float temp_c);
void adc_init_calibration(void);
float convert_to_percentage(float Batt_volt);


#endif /* COMPONENTS_TEMPERATURE_JIRS40_H_ */
