/*
 * config.h
 * Project-wide compile-time configuration.
 * All pin numbers, addresses, and tuning knobs live here.
 */

#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include "driver/gpio.h"

/* ── Debug ───────────────────────────────────────────────────── */
#define DEBUG_LOGS              0   /* 1 = verbose sensor prints  */

/* ── Battery ─────────────────────────────────────────────────── */
#define BATT_AVG_SAMPLES        16  /* rolling average window     */
#define ADC_BATT_GAIN           1.033f

/* ── CPU stats buffer ────────────────────────────────────────── */
#define STATS_BUFFER_SIZE       1024

/* ── BNO085 feature timing ───────────────────────────────────── */
#define STEP_FREQ_UPDATE_WIN_SEC  2.0f
#define VAR_WINDOW_SIZE           100   /* 1 s @ 100 Hz            */

/* ── FFT / Respiration ───────────────────────────────────────── */
#define FFT_SIZE                1024
#define SAMPLE_RATE             100.0f  /* Hz                      */
#define RESP_FREQ_MIN_HZ        0.15f
#define RESP_FREQ_MAX_HZ        0.80f
#define RESP_MAG_THRESHOLD      0.01f

#endif /* COMMON_CONFIG_H_ */
