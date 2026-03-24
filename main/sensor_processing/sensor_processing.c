/*
 * sensor_processing.c
 *
 * Computes derived quantities from raw sensor data.
 * Refactored from:
 *   - main/sensor_data_processing/data_processing.c
 *   - components/imu/bno085.c  (FFT, ROM, respiration helpers)
 */

#include "sensor_processing.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <esp_dsp.h>

static const char *TAG = "SENSOR_PROC";

/* ── FFT rolling buffer ──────────────────────────────────────── */
static float  s_fft_buf[FFT_SIZE];
static int    s_fft_idx   = 0;
static float  s_resp_rate = 0.0f;   /* persists between calls */

/* ── ROM integration ─────────────────────────────────────────── */
static float    s_integrated_angle = 0.0f;
static uint64_t s_last_gyro_ms     = 0;
static uint64_t s_last_rom_reset   = 0;

/* ── Init ──────────────────────────────────────────────────────── */
void sensor_processing_init(void)
{
    dsps_fft2r_init_fc32(NULL, FFT_SIZE);
    ESP_LOGI(TAG, "Sensor processing initialised (FFT size=%d)", FFT_SIZE);
}

/* ── Magnitude helpers ─────────────────────────────────────────── */
static float vec3_magnitude(float x, float y, float z)
{
    return sqrtf(x*x + y*y + z*z);
}

/* ── Range-of-motion via gyro integration ──────────────────────── */
static void update_rom(const sensor_raw_t *r, sensor_processed_t *out)
{
    uint64_t now_ms = esp_timer_get_time() / 1000ULL;
    if (s_last_gyro_ms > 0) {
        float dt = (now_ms - s_last_gyro_ms) / 1000.0f;
        float speed = vec3_magnitude(r->gyro_x, r->gyro_y, r->gyro_z);
        /* gyro is in deg/s; convert to radians for integration */
        s_integrated_angle += (speed * (float)M_PI / 180.0f) * dt;

        if ((now_ms - s_last_rom_reset) > 5000) {
            out->range_of_motion = fminf((s_integrated_angle / (float)M_PI) * 100.0f, 100.0f);
            s_integrated_angle  = 0.0f;
            s_last_rom_reset    = now_ms;
        }
    }
    s_last_gyro_ms = now_ms;
}

/* ── Main processing tick ──────────────────────────────────────── */
void sensor_processing_run(const sensor_raw_t *raw, sensor_processed_t *out)
{
    /* Magnitudes */
    out->accel_magnitude = vec3_magnitude(raw->accel_x, raw->accel_y, raw->accel_z);
    out->gyro_magnitude  = vec3_magnitude(raw->gyro_x,  raw->gyro_y,  raw->gyro_z);
    out->mag_magnitude   = vec3_magnitude(raw->mag_x,   raw->mag_y,   raw->mag_z);

    /* acc_variance is computed in mw_bno085 circular buffer and
       exposed via raw->acc_variance. Copy it straight through. */
    out->acc_variance = raw->acc_variance;

    /* Range of motion */
    update_rom(raw, out);

    /* Carry through respiration from last FFT call */
    out->respiration_rate = s_resp_rate;
}

/* ── FFT respiration tick ──────────────────────────────────────── */
void sensor_processing_fft_tick(const sensor_raw_t *raw, sensor_processed_t *out)
{
    float mag = vec3_magnitude(raw->lin_accel_x, raw->lin_accel_y, raw->lin_accel_z);
    s_fft_buf[s_fft_idx++] = mag;

    if (s_fft_idx < FFT_SIZE) return;  /* buffer not full yet */

    /* ── Buffer full: run FFT ─────────────────────────────────── */
    static float fft_work[FFT_SIZE * 2];
    static float magnitude[FFT_SIZE / 2];

    /* Remove DC */
    float mean = 0;
    for (int i = 0; i < FFT_SIZE; i++) mean += s_fft_buf[i];
    mean /= FFT_SIZE;

    for (int i = 0; i < FFT_SIZE; i++) {
        fft_work[2*i]   = s_fft_buf[i] - mean;
        fft_work[2*i+1] = 0.0f;
    }

    dsps_fft2r_fc32(fft_work, FFT_SIZE);
    dsps_bit_rev_fc32(fft_work, FFT_SIZE);

    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float re = fft_work[2*i];
        float im = fft_work[2*i+1];
        magnitude[i] = sqrtf(re*re + im*im);
    }

    /* Find dominant frequency in respiration band */
    float best_mag = 0, best_freq = 0;
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        float freq = (i * SAMPLE_RATE) / FFT_SIZE;
        if (freq < RESP_FREQ_MIN_HZ || freq > RESP_FREQ_MAX_HZ) continue;
        if (magnitude[i] > best_mag && magnitude[i] > RESP_MAG_THRESHOLD) {
            best_mag  = magnitude[i];
            best_freq = freq;
        }
    }

    if (best_freq > 0) {
        float rr = best_freq * 60.0f;
        s_resp_rate = (s_resp_rate == 0) ? rr
                      : s_resp_rate + 0.1f * (rr - s_resp_rate);
        out->respiration_rate = s_resp_rate;
    }

    /* 75 % overlap: shift buffer */
    int shift = FFT_SIZE / 4;
    for (int i = 0; i < FFT_SIZE - shift; i++)
        s_fft_buf[i] = s_fft_buf[i + shift];
    s_fft_idx = FFT_SIZE - shift;
}
