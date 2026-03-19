#include <stdio.h>
#include <math.h>
#include "../../components/common/config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "max30102.h"
#include "string.h"
#include "../../main/Feature_processing/feature_processing.h"
 
static const char *TAG = "HEART_RATE";
static i2c_master_dev_handle_t max_dev;
 
/* ================= CONFIG ================= */
#define SAMPLE_RATE_HZ        100
 
#define IR_DC_THRESHOLD      50000
#define IR_AC_THRESHOLD      600
 
#define MIN_IBI_MS           400    // 150 BPM
#define MAX_IBI_MS           1500   // 40 BPM
 
#define IBI_BUF_SIZE         5
#define HR_RATE_LIMIT        1      // BPM/sec
 
#define HR_PUBLISH_INTERVAL_MS 250
 
#define DC_ALPHA 0.995f  // ~5 sec time constant
 
#define SQI_MIN_ACDC        0.0010f   // 0.17%
#define SQI_GOOD_ACDC      0.0013f   // 0.10%
#define SQI_MAX_ACDC      0.05f // 0.0030f //SKC
#define SQI_GOOD_REQUIRED  2         // windows
#define SQI_BAD_ALLOWED    2         // tolerance
 
 
#define HR_AVG_ENABLE        1      // 0 = off, 1 = on
#define HR_AVG_WINDOW        25      // beats to average
#define HR_HOLD_MS           3000   // hold last HR if signal dips
#define HR_SMOOTH_ALPHA      0.25f  // EMA smoothing (0.1–0.3 good)
#define BPM_MA_WINDOW 8   // 6–10 moving average window size
 

static uint32_t last_valid_bpm = 0;
static uint32_t last_finger_seen_ms = 0;
static bool finger_detected_prev = false;


/*====MA Filter helper=======*/
 
static uint32_t bpm_moving_avg(uint32_t *buf, uint8_t cnt)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < cnt; i++)
        sum += buf[i];
    return sum / cnt;
}
 
static uint32_t avg_u32(uint32_t *buf, uint8_t n)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++)
        sum += buf[i];
    return sum / n;
}

/* ================= TIME ================= */
static inline uint32_t millis(void)
{
    return esp_timer_get_time() / 1000;
}
 
/* ================= I2C ================= */
esp_err_t max30102_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_transmit(max_dev, buf, sizeof(buf), -1);
}
 
esp_err_t max30102_read_fifo(uint8_t *data, size_t len)
{
    uint8_t reg = REG_FIFO_DATA;
    return i2c_master_transmit_receive(max_dev, &reg, 1, data, len, -1);
}
 
 
esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(
        max_dev,
        &reg,
        1,
        data,
        1,
        -1
    );
}

/* ================= INIT ================= */
void max30102_init(i2c_master_dev_handle_t dev)
{
    max_dev = dev;
 
    max30102_write(REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));
 
    max30102_write(REG_FIFO_WR_PTR, 0);
    max30102_write(REG_OVF_COUNTER, 0);
    max30102_write(REG_FIFO_RD_PTR, 0);
 
    // FIFO config: NO interrupt dependency
    max30102_write(REG_FIFO_CONFIG, 0x4F);
 
    // Disable interrupts
    max30102_write(REG_INTR_ENABLE_1, 0x00);
    max30102_write(REG_INTR_ENABLE_2, 0x00);
	
    max30102_write(REG_MODE_CONFIG, 0x02);   // HR only
    max30102_write(REG_SPO2_CONFIG, 0x2F);   // 100Hz, 18-bit
    max30102_write(REG_LED1_PA, 0x28);       // IR LED
    max30102_write(REG_LED2_PA, 0x00);
 
    ESP_LOGI("MAX30102", "Initialized @100Hz (polling)");
}
 
/* ================= PEAK DETECTOR ================= */
void hr_process_sample(uint32_t ir)
{
    /* ---------------- State ---------------- */
    static float dc_mean = 0.0f;
    static float ac_sq_sum = 0.0f;
    static uint16_t rms_cnt = 0;
    static float ac_rms = 0.0f;
    
    /* Band-pass state - IMPROVED for 100Hz */
    static float hp_y = 0.0f;
    static float hp_x_prev = 0.0f;
    static float lp_y = 0.0f;
    
    /* Peak detection */
    static float prev_lp = 0.0f;
    static float prev_diff = 0.0f;
    static float peak_threshold = 0.0f;  // ADAPTIVE threshold
    
    static uint64_t last_peak_time = 0;
    static uint64_t refractory_until = 0;
    
    /* IBI median filter - EXPANDED */
    static uint64_t ibi_hist[5] = {0};  // More history = better filtering
    static uint8_t  ibi_idx = 0;
    static uint8_t  ibi_count = 0;
    
    /* BPM smoothing */
    static uint32_t bpm_smooth = 0;
    static uint8_t valid_peak_count = 0;
    
    /* Signal quality and motion detection */
    static float ir_raw_prev = 0.0f;
    static float ir_variance = 0.0f;
    static uint16_t motion_samples = 0;
    static bool finger_detected = false;
    
	/* ---------- HRV (RMSSD) ---------- */
	static float rr_prev_ms = 0.0f;
	static float hrv_sum_sq = 0.0f;
	static uint8_t hrv_count = 0;
    
    /* ---------- DC removal (FASTER adaptation) ---------- */
    dc_mean += 0.02f * ((float)ir - dc_mean);  // Was 0.01, now faster
    float ac = (float)ir - dc_mean;
    
    /* ---------- Motion Detection & Signal Quality ---------- */
    // Track raw IR variance to detect motion/noise
    float ir_diff = (float)ir - ir_raw_prev;
    ir_variance += 0.1f * (fabsf(ir_diff) - ir_variance);
    ir_raw_prev = (float)ir;
    motion_samples++;
    
    // Check every 100 samples (1 second at 100Hz)
    if (motion_samples >= 100) {
        bool strong_signal = (dc_mean > 12000.0f && dc_mean < 120000);
        bool low_motion = (ir_variance < 500.0f);  // Tune this threshold
        bool has_pulse = (ac_rms > 40.0f);  // White surfaces have weak AC
        
        finger_detected = strong_signal && low_motion && has_pulse;
#if DEBUG_LOGS		 
        if (!finger_detected) {
			
            ESP_LOGW("HR", "No valid finger: DC=%.0f, Variance=%.1f, RMS=%.1f %s%s%s", 
                     dc_mean, ir_variance, ac_rms,
                     strong_signal ? "" : "[Weak DC] ",
                     low_motion ? "" : "[Motion] ",
                     has_pulse ? "" : "[No pulse]");
        }
#endif
        
        motion_samples = 0;
    }
    
	    /* ---------- Band-pass (TUNED for 100Hz, 0.5-5Hz passband) ---------- */
	    // High-pass: 0.5 Hz cutoff
	    float hp = 0.985f * (hp_y + ac - hp_x_prev);  // Was 0.969
	    hp_x_prev = ac;
	    hp_y = hp;
	    
	    // Low-pass: 5 Hz cutoff
	    lp_y += 0.27f * (hp - lp_y);  // Was 0.201, now smoother
	    
    /* ========== PLOTTING OUTPUT ========== */
#if PLOT_AC_SIGNAL
    // Plot the filtered AC signal for Serial Plotter
    printf("%f\n", lp_y);
    // Optionally plot multiple signals (uncomment to see all):
    // printf("AC:%f,Filtered:%f,Threshold:%f\n", ac, lp_y, peak_threshold);
#endif
    /* ==================================== */
    /*---------- RMS (for signal quality) ---------- */
    ac_sq_sum += lp_y * lp_y;
    rms_cnt++;
    if(rms_cnt >= RMS_WINDOW_SAMPLES) {
        ac_rms = sqrtf(ac_sq_sum / rms_cnt);  
        // ADAPTIVE threshold based on recent signal strength
        peak_threshold = ac_rms * 0.6f;  // More conservative than 1.5x
        ac_sq_sum = 0;
        rms_cnt = 0;
    }
	
    /* ---------- Peak detection (IMPROVED) ---------- */
    float diff = lp_y - prev_lp;
    uint64_t now = esp_timer_get_time();
    
    // Wait for: good signal quality + finger detected + refractory period
    if (finger_detected && ac_rms >= 10.0f && now >= refractory_until) {
        
        // Peak = zero-crossing on derivative + amplitude check
        bool peak = (prev_diff > 0.0f) && 
                    (diff <= 0.0f) && 
                    (lp_y > peak_threshold);
        
        if (peak) {
            valid_peak_count++;
            
#if DEBUG_LOGS			 
            // Log EVERY peak for diagnostics
            ESP_LOGI("PEAK", "Peak #%u detected, lp_y=%.1f, threshold=%.1f", 
                     valid_peak_count, lp_y, peak_threshold);
#endif
            
            if (last_peak_time != 0) {
                uint64_t ibi_us = now - last_peak_time;
                
                // Physiological limits: 40-200 BPM (300-1500ms)
                if (ibi_us > 300000 && ibi_us < 1500000) {
                    
                    // Store in circular buffer
                    ibi_hist[ibi_idx++] = ibi_us;
                    ibi_idx %= 5;
                    
                    if (ibi_count < 5) {
                        ibi_count++;
                    }
                    
                    // Calculate median once we have enough samples
                    if (ibi_count >= 3) {
                        // Simple median of 5 values
                        uint64_t sorted[5];
                        memcpy(sorted, ibi_hist, sizeof(sorted));
                        
                        // Bubble sort (fine for 5 elements)
                        for (int i = 0; i < ibi_count - 1; i++) {
                            for (int j = 0; j < ibi_count - i - 1; j++) {
                                if (sorted[j] > sorted[j+1]) {
                                    uint64_t temp = sorted[j];
                                    sorted[j] = sorted[j+1];
                                    sorted[j+1] = temp;
                                }
                            }
                        }
                        
                        uint64_t ibi_med = sorted[ibi_count / 2];
                        uint32_t bpm = 60000000ULL / ibi_med;
						
						
						/* ---------- HRV Calculation ---------- */
						float rr_ms = (float)ibi_med / 1000.0f;   // convert us → ms

						if (rr_prev_ms > 0.0f)
						{
						    float diff_rr = rr_ms - rr_prev_ms;
						    hrv_sum_sq += diff_rr * diff_rr;
						    hrv_count++;

						    if (hrv_count >= HRV_WINDOW)
						    {
						        sm.sensor_data.hrv = sqrtf(hrv_sum_sq / hrv_count);  // RMSSD
								sm.last_hrv=sm.sensor_data.hrv;
						        hrv_sum_sq = 0.0f;
						        hrv_count = 0;
						    }
						}

						rr_prev_ms = rr_ms;
                        
                        // Apply BPM limits
                        if (bpm >= 40 && bpm <= 200) {
                            
                            // HEAVIER smoothing for stability
                            if (bpm_smooth == 0) {
                                bpm_smooth = bpm;
                            } else {
                                bpm_smooth = (bpm_smooth * 7 + bpm) / 8;  // Was 3/4
                            }
							
							 sm.last_heart_rate = bpm_smooth;
							 sm.sensor_data.heart_rate = bpm_smooth;
							 //bpm_global = bpm_smooth;
							 last_valid_bpm = bpm_smooth;
							 last_finger_seen_ms = millis();

                            
#if DEBUG_LOGS
                            // Log with IBI for debugging
							ESP_LOGI("HR", "BPM=%lu HRV=%.2fms IBI=%llums AC_RMS=%.1f",
							         bpm_smooth, sm.sensor_data.hrv, ibi_med/1000, ac_rms);
#endif              
						}
                        //bpm_global = 0;
                    }
#if DEBUG_LOGS
					 
                } else {
                    ESP_LOGW("HR", "IBI out of range: %llu ms", ibi_us/1000);
#endif
                }
            }
            
            // Update timing
            last_peak_time = now;
            
            // ADAPTIVE refractory based on expected HR
            // At 82 BPM, period is ~730ms, so 300ms refractory = ~40% of cycle
            if (bpm_smooth > 0) {
                uint64_t expected_period = 60000000ULL / bpm_smooth;
                refractory_until = now + (expected_period / 3);  // 33% of cycle
            } else {
                refractory_until = now + 300000;  // Default 300ms
            }
        }
    }
    
    /* ---------- Reset state when finger removed ---------- */
	   if (!finger_detected) {
	    ibi_count = 0;
	    valid_peak_count = 0;
	    last_peak_time = 0;
		//Reset Hrv variables if finger is not detected
		sm.sensor_data.hrv = 0.0f;
		rr_prev_ms = 0.0f;
		hrv_sum_sq = 0.0f;
		hrv_count = 0;
	    // DO NOT touch bpm_smooth or bpm_global here
	}
	
	
	uint32_t now_ms = millis();
	if (!finger_detected) {
	    // Hold last BPM for 10 seconds
	    if ((now_ms - last_finger_seen_ms) >= 10000) {
	        sm.sensor_data.heart_rate = 0;
			sm.last_heart_rate = 0;
	    } else {
	        sm.sensor_data.heart_rate = last_valid_bpm;
			sm.last_heart_rate = last_valid_bpm;
	    }
	}
	
	if (finger_detected && !finger_detected_prev) {
    // Finger just placed
    last_finger_seen_ms = millis();
	}
	finger_detected_prev = finger_detected;

	
    
    prev_diff = diff;
    prev_lp = lp_y;
}