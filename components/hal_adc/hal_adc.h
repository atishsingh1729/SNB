/*
 * hal_adc.h
 * HAL Layer - ADC abstraction
 * Provides raw ADC reads used by NTC temperature middleware.
 */

#ifndef HAL_ADC_H_
#define HAL_ADC_H_

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* ── Unit & channels ─────────────────────────────────────────── */
#define HAL_ADC_UNIT          ADC_UNIT_1
#define HAL_ADC_CH_BODY       ADC_CHANNEL_0   /* GPIO1  – body NTC   */
#define HAL_ADC_CH_AMB        ADC_CHANNEL_6   /* GPIO7  – ambient NTC */
#define HAL_ADC_CH_BATT       ADC_CHANNEL_5   /* GPIO6  – battery    */
#define HAL_ADC_CH_AUDIO      ADC_CHANNEL_6   /* GPIO7  – microphone  */

/* ── Attenuation ─────────────────────────────────────────────── */
#define HAL_ADC_ATTEN_TEMP    ADC_ATTEN_DB_12
#define HAL_ADC_ATTEN_BATT    ADC_ATTEN_DB_2_5

/* ── Public handles (owned by hal_adc.c) ─────────────────────── */
extern adc_oneshot_unit_handle_t hal_adc_handle;
extern adc_cali_handle_t         hal_adc_cali_temp;
extern adc_cali_handle_t         hal_adc_cali_batt;

/* ── API ──────────────────────────────────────────────────────── */
void  hal_adc_init(void);
void  hal_adc_init_calibration(void);

/**
 * @brief  Read a calibrated voltage from one ADC channel.
 * @param  channel      ADC channel enum.
 * @param  cali_handle  Calibration handle matching the channel's attenuation.
 * @return Voltage in Volts (float).
 */
float hal_adc_read_voltage(adc_channel_t channel, adc_cali_handle_t cali_handle);

#endif /* HAL_ADC_H_ */
