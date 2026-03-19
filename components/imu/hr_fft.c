/*
 * hr_fft.c
 *
 *  Created on: Jan 29, 2026
 *      Author: Sanjay.Chauhan
 */

 #include "hr_fft.h"
 #include <math.h>
 #include <string.h>
 #include "esp_dsp.h"
 #include <stdio.h>
 #include <math.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_log.h"
 #include "esp_timer.h"
 #include "driver/i2c_master.h"
 #include <inttypes.h>
 #include "max30102.h"
 #include "../../main/Feature_processing/feature_processing.h"
 #include "string.h"
 //#include "hr_fft.h"
 
 
 #define SAMPLE_RATE_HZ   100
 #define FFT_SIZE         256

 #define HR_MIN_BPM       40
 #define HR_MAX_BPM       180

 static float fft_input[FFT_SIZE];
 static float fft_window[FFT_SIZE];
 static float fft_out[FFT_SIZE];
 static float fft_mag[FFT_SIZE / 2];

 static int sample_idx = 0;
 static uint8_t fft_ready = 0;

 static float bpm_ema = 0.0f;
 #define BPM_EMA_ALPHA  0.2f

 void hr_fft_init(void)
 {
     dsps_fft2r_init_fc32(NULL, FFT_SIZE);
     dsps_wind_hann_f32(fft_window, FFT_SIZE);
 }

 void hr_fft_push_sample(float ir_ac)
 {
     fft_input[sample_idx++] = ir_ac;

     if (sample_idx >= FFT_SIZE) {
         sample_idx = 0;
         fft_ready = 1;
     }
 }

 uint32_t hr_fft_get_bpm(void)
 {
     if (!fft_ready)
         return 0;

     fft_ready = 0;

     // Apply window
     for (int i = 0; i < FFT_SIZE; i++) {
         fft_out[i] = fft_input[i] * fft_window[i];
     }

     // FFT
     dsps_fft2r_fc32(fft_out, FFT_SIZE);
     dsps_bit_rev_fc32(fft_out, FFT_SIZE);
     dsps_cplx2reC_fc32(fft_out, FFT_SIZE);

     // Magnitude
     for (int i = 1; i < FFT_SIZE / 2; i++) {
         fft_mag[i] = sqrtf(
             fft_out[2*i] * fft_out[2*i] +
             fft_out[2*i+1] * fft_out[2*i+1]
         );
     }

     // Find HR peak
     float max_mag = 0.0f;
     int max_bin = 0;

     for (int i = 1; i < FFT_SIZE / 2; i++) {
         float freq = (float)i * SAMPLE_RATE_HZ / FFT_SIZE;
         float bpm = freq * 60.0f;

         if (bpm < HR_MIN_BPM || bpm > HR_MAX_BPM)
             continue;

         if (fft_mag[i] > max_mag) {
             max_mag = fft_mag[i];
             max_bin = i;
         }
     }

     if (max_bin == 0)
         return 0;

     float hr_bpm = ((float)max_bin * SAMPLE_RATE_HZ * 60.0f) / FFT_SIZE;

     // EMA smoothing
     if (bpm_ema == 0.0f)
         bpm_ema = hr_bpm;
     else
         bpm_ema = (BPM_EMA_ALPHA * hr_bpm) +
                   ((1.0f - BPM_EMA_ALPHA) * bpm_ema);

     return (uint32_t)(bpm_ema + 0.5f);
 }

 
 


