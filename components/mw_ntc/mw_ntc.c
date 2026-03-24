/*
 * mw_ntc.c
 * Middleware – NTC thermistor + battery ADC.
 * Refactored from components/temperature/jirs40.c
 */

#include "mw_ntc.h"
#include "hal_adc.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MW_NTC";

/* ── Battery stabilisation ────────────────────────────────────
 *
 * Problem: a 16-sample × 3 s rolling average = only 48 s window.
 * Plugging/unplugging a charger changes voltage by 0.3–0.5 V instantly
 * → percentage jumps 20–40 % in one update cycle.
 *
 * Solution: two-stage filter
 *   Stage 1 – voltage EMA (α = 0.15, τ ≈ 18 s): smooths ADC noise
 *             and the step from connecting a charger.
 *   Stage 2 – rate limiter on the displayed percentage:
 *             output is allowed to change at most ±BATT_MAX_STEP_PCT
 *             per call (= per 3 s update).  At 0.5 %/update a
 *             20 % swing takes 40 s to reflect — appropriate for a
 *             battery gauge.  A real 1 % drain at 3 s/update would
 *             update at roughly the right rate.
 *
 * No circular buffer needed; uses three static floats only.
 */
#define BATT_V_EMA_ALPHA    0.15f   /* voltage EMA alpha              */
#define BATT_MAX_STEP_PCT   0.5f    /* max %/update shown to user     */

static float s_batt_v_ema   = 0.0f; /* smoothed voltage               */
static float s_batt_pct_out = -1.0f;/* last displayed %, -1 = uninit */

/* ── Init ──────────────────────────────────────────────────────── */
void mw_ntc_init(void)
{
    /* ADC and calibration already done by hal_adc_init() */
    ESP_LOGI(TAG, "NTC middleware ready");
}

/* ── Conversions ───────────────────────────────────────────────── */
float mw_ntc_voltage_to_resistance(float vout)
{
    if (vout <= 0.01f || vout >= (NTC_VREF - 0.01f)) return -1.0f;
    return (NTC_SERIES_R * vout) / (NTC_VREF - vout);
}

float mw_ntc_resistance_to_temp(float r)
{
    float tk = 1.0f / ((1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(r / NTC_R0));
    return tk - 273.15f;
}

float mw_ntc_apply_calibration(float temp_c)
{
    return (NTC_CAL_SLOPE * temp_c) + NTC_CAL_OFFSET;
}

float mw_ntc_voltage_to_battery_percent(float batt_v)
{
    if (batt_v >= 4.20f) return 100.0f;
    if (batt_v <= 3.27f) return   0.0f;

    static const float v_tbl[] = {
        4.20f, 4.15f, 4.11f, 4.08f, 4.02f,
        3.98f, 3.95f, 3.91f, 3.87f, 3.85f,
        3.84f, 3.82f, 3.80f, 3.79f, 3.77f,
        3.75f, 3.73f, 3.71f, 3.69f, 3.61f, 3.27f
    };
    static const float p_tbl[] = {
        100, 95, 90, 85, 80,
         75, 70, 65, 60, 55,
         50, 45, 40, 35, 30,
         25, 20, 15, 10,  5,  0
    };
    int n = sizeof(v_tbl) / sizeof(v_tbl[0]);
    for (int i = 0; i < n - 1; i++) {
        if (batt_v <= v_tbl[i] && batt_v >= v_tbl[i+1]) {
            float frac = (batt_v - v_tbl[i]) / (v_tbl[i+1] - v_tbl[i]);
            return p_tbl[i] + frac * (p_tbl[i+1] - p_tbl[i]);
        }
    }
    return 0.0f;
}

/* ── Public get (called by raw_data_reader in LIVE mode) ─────── */
void mw_ntc_get_raw(sensor_raw_t *out)
{
    /* Body temperature */
    float vbody = hal_adc_read_voltage(HAL_ADC_CH_BODY, hal_adc_cali_temp);
    float r_body = mw_ntc_voltage_to_resistance(vbody);
    float t_body = mw_ntc_apply_calibration(mw_ntc_resistance_to_temp(r_body));
    out->body_temp = t_body;

    /* Ambient temperature */
    float vamb = hal_adc_read_voltage(HAL_ADC_CH_AMB, hal_adc_cali_temp);
    float r_amb = mw_ntc_voltage_to_resistance(vamb);
    float t_amb = mw_ntc_apply_calibration(mw_ntc_resistance_to_temp(r_amb));
    out->ambient_temp = t_amb;

    /* Battery — two-stage filter */
    float div_v  = hal_adc_read_voltage(HAL_ADC_CH_BATT, hal_adc_cali_batt) * ADC_BATT_GAIN;
    float batt_v = div_v * 3.57f;
    out->batt_voltage = batt_v;

    /* Stage 1: voltage EMA — smooths ADC noise and charger step */
    if (s_batt_v_ema == 0.0f)
        s_batt_v_ema = batt_v;   /* seed on first call */
    else
        s_batt_v_ema += BATT_V_EMA_ALPHA * (batt_v - s_batt_v_ema);

    /* Stage 2: convert smoothed voltage → %, then rate-limit display */
    float target_pct = mw_ntc_voltage_to_battery_percent(s_batt_v_ema);

    if (s_batt_pct_out < 0.0f) {
        /* First call — initialise directly so display isn't stuck at 0 */
        s_batt_pct_out = target_pct;
    } else {
        float delta = target_pct - s_batt_pct_out;
        if (delta >  BATT_MAX_STEP_PCT) delta =  BATT_MAX_STEP_PCT;
        if (delta < -BATT_MAX_STEP_PCT) delta = -BATT_MAX_STEP_PCT;
        s_batt_pct_out += delta;
    }

    out->batt_percent = s_batt_pct_out;
}
