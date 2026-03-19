/*
 * jirs40.c
 *
 *  Created on: Dec 24, 2025
 *      Author: Dheeraj.Jain
 */

#include "jirs40.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "math.h"

adc_oneshot_unit_handle_t adc_handle;

adc_cali_handle_t adc_cali_temp = NULL;
adc_cali_handle_t adc_cali_batt = NULL;







void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_USED,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    /* -------- Temperature Channels (11dB) -------- */
    adc_oneshot_chan_cfg_t temp_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CH_BODY, &temp_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CH_AMB,  &temp_cfg));

    /* -------- Battery Channel (2.5dB) -------- */
    adc_oneshot_chan_cfg_t batt_cfg = {
        .atten = ADC_ATTEN_DB_2_5,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CH_BATT, &batt_cfg));
}

void adc_init_calibration(void)
{
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_USED,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    /* -------- Temp Calibration (12dB) -------- */
    cali_cfg.atten = ADC_ATTEN_DB_12;

    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_temp) == ESP_OK) {
        printf("Temp ADC calibration OK\n");
    }

    /* -------- Battery Calibration (2.5dB) -------- */
    cali_cfg.atten = ADC_ATTEN_DB_2_5;

    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_batt) == ESP_OK) {
        printf("Battery ADC calibration OK\n");
    }
}

float adc_read(adc_channel_t channel, adc_cali_handle_t cali_handle)
{
    int raw = 0;
    int sum = 0;
    int voltage_mv = 0;

    for (int i = 0; i < 16; i++) {
        adc_oneshot_read(adc_handle, channel, &raw);
        sum += raw;
        esp_rom_delay_us(20);   // microseconds only
    }

    raw = sum / 16;

    if (cali_handle) {
        adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
        return voltage_mv / 1000.0f;
    }

    return 0.0f;
}


float ntc_voltage_to_resistance(float vout)
{
    if (vout <= 0.01f || vout >= (ADC_VREF - 0.01f))
        return -1.0f;

    return (SERIES_RESISTOR * vout) / (ADC_VREF - vout);
}


float ntc_resistance_to_temp(float r_ntc)
{
    float temp_k;

    temp_k = 1.0f / (
        (1.0f / NTC_T0) +
        (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0)
    );

    return temp_k - 273.15f;   // Kelvin → Celsius
}

float ntc_apply_calibration(float temp_c)
{
    return (NTC_CAL_SLOPE * temp_c) + NTC_CAL_OFFSET_C;
}


float convert_to_percentage(float Batt_volt)
{
    if (Batt_volt >= 4.20f)
        return 100.0f;

    if (Batt_volt <= 3.27f)
        return 0.0f;

    const float voltage_table[] = {
        4.20, 4.15, 4.11, 4.08, 4.02,
        3.98, 3.95, 3.91, 3.87, 3.85,
        3.84, 3.82, 3.80, 3.79, 3.77,
        3.75, 3.73, 3.71, 3.69, 3.61, 3.27
    };

    const float percent_table[] = {
        100, 95, 90, 85, 80,
        75, 70, 65, 60, 55,
        50, 45, 40, 35, 30,
        25, 20, 15, 10, 5, 0
    };

    int table_size = sizeof(voltage_table) / sizeof(voltage_table[0]);

    for (int i = 0; i < table_size - 1; i++)
    {
        if (Batt_volt <= voltage_table[i] &&
            Batt_volt >= voltage_table[i + 1])
        {
            float v1 = voltage_table[i];
            float v2 = voltage_table[i + 1];
            float p1 = percent_table[i];
            float p2 = percent_table[i + 1];

            return p1 + (Batt_volt - v1) * (p2 - p1) / (v2 - v1);
        }
    }

    return 0.0f;
}