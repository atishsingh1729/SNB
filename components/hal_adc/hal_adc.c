/*
 * hal_adc.c
 * HAL Layer – ADC init + calibrated read.
 * No sensor logic here; just hardware abstraction.
 */

#include "hal_adc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "HAL_ADC";

adc_oneshot_unit_handle_t hal_adc_handle    = NULL;
adc_cali_handle_t         hal_adc_cali_temp = NULL;
adc_cali_handle_t         hal_adc_cali_batt = NULL;

/* ── Init ──────────────────────────────────────────────────────── */
void hal_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = HAL_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &hal_adc_handle));

    /* Temperature channels – 12 dB */
    adc_oneshot_chan_cfg_t temp_cfg = {
        .atten    = HAL_ADC_ATTEN_TEMP,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_adc_handle, HAL_ADC_CH_BODY, &temp_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_adc_handle, HAL_ADC_CH_AMB,  &temp_cfg));

    /* Battery channel – 2.5 dB */
    adc_oneshot_chan_cfg_t batt_cfg = {
        .atten    = HAL_ADC_ATTEN_BATT,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_adc_handle, HAL_ADC_CH_BATT, &batt_cfg));

    ESP_LOGI(TAG, "ADC unit initialised");
}

void hal_adc_init_calibration(void)
{
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = HAL_ADC_UNIT,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    /* Temp calibration – 12 dB */
    cali_cfg.atten = HAL_ADC_ATTEN_TEMP;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &hal_adc_cali_temp) == ESP_OK)
        ESP_LOGI(TAG, "Temp ADC calibration OK");

    /* Battery calibration – 2.5 dB */
    cali_cfg.atten = HAL_ADC_ATTEN_BATT;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &hal_adc_cali_batt) == ESP_OK)
        ESP_LOGI(TAG, "Battery ADC calibration OK");
}

/* ── Read ──────────────────────────────────────────────────────── */
float hal_adc_read_voltage(adc_channel_t channel, adc_cali_handle_t cali_handle)
{
    int raw = 0, sum = 0, voltage_mv = 0;

    for (int i = 0; i < 16; i++) {
        adc_oneshot_read(hal_adc_handle, channel, &raw);
        sum += raw;
        esp_rom_delay_us(20);
    }
    raw = sum / 16;

    if (cali_handle) {
        adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
        return voltage_mv / 1000.0f;
    }
    return 0.0f;
}
