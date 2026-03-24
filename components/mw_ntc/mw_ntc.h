/*
 * mw_ntc.h
 * Middleware – NTC thermistor + battery ADC.
 * Sits above hal_adc; provides calibrated temperatures and battery %.
 */

#ifndef MW_NTC_H_
#define MW_NTC_H_

#include "sensor_types.h"

/* ── NTC parameters ──────────────────────────────────────────── */
#define NTC_VREF            3.289f
#define NTC_R0              10000.0f   /* 10 kΩ nominal           */
#define NTC_T0              298.15f    /* 25 °C in Kelvin         */
#define NTC_BETA            3950.0f
#define NTC_SERIES_R        10000.0f   /* voltage-divider resistor */
#define NTC_CAL_SLOPE       1.216f
#define NTC_CAL_OFFSET      -5.37f
#define ADC_BATT_GAIN       1.033f     /* hardware gain correction */

/* ── API ─────────────────────────────────────────────────────── */
void  mw_ntc_init(void);

/**
 * @brief  Read body temp, ambient temp, battery voltage + % into *out.
 *         Called by raw_data_reader in LIVE mode.
 */
void  mw_ntc_get_raw(sensor_raw_t *out);

/* Exposed helpers (useful for unit testing / calibration) */
float mw_ntc_voltage_to_resistance(float vout);
float mw_ntc_resistance_to_temp(float r);
float mw_ntc_apply_calibration(float temp_c);
float mw_ntc_voltage_to_battery_percent(float batt_v);

#endif /* MW_NTC_H_ */
