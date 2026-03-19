/*
 * mp23abs1tr.c
 *
 *  Created on: Dec 24, 2025
 *      Author: Dheeraj.Jain
 */

#include "mp23abs1tr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "../../main/Feature_processing/feature_processing.h"

static const char *TAG = "AUDIO";

extern adc_oneshot_unit_handle_t adc_handle;
extern adc_cali_handle_t adc_cali_handle;

float mp23abs1_read_rms_mv(void){
    int raw, mv;
    float mean = 0.0f;
    float sum_sq = 0.0f;

    /* Measure DC bias */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_AUDIO, &raw);
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv);
        mean += mv;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_US));
    }
    mean /= SAMPLE_COUNT;
	printf("Mic DC Bias = %.3f V\n", mean / 1000.0f);
    /* RMS of AC component */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_AUDIO, &raw);
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv);
        float ac = mv - mean;
        sum_sq += ac * ac;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_US));
    }

    return sqrtf(sum_sq / SAMPLE_COUNT);
}

// Helper function to estimate dominant frequency (simplified)
float estimate_dominant_frequency(float* buffer, int length) {
    // In real implementation, use FFT
    // This is a simplified version for demonstration
    
    static int zero_crossings = 0;
    static float last_sample = 0;
    
    // Count zero crossings for rough frequency estimation
    for (int i = 0; i < length; i++) {
        if ((last_sample < 0 && buffer[i] >= 0) || 
            (last_sample >= 0 && buffer[i] < 0)) {
            zero_crossings++;
        }
        last_sample = buffer[i];
    }
    
    // Calculate approximate frequency (zero crossings per second)
    float time_window = length / 44100.0f;  // Assuming 44.1kHz sample rate
    return zero_crossings / (2.0f * time_window);
}

// Calculate pattern regularity (for distinguishing crunch vs lap)
float calculate_pattern_regularity(float* buffer, int length) {
    // Analyze autocorrelation or envelope regularity
    // Simplified version - checks variance in amplitude peaks
    
    float peak_intervals[20];
    int peak_count = 0;
    float threshold = 0.5f;  // Amplitude threshold for peaks
    
    for (int i = 1; i < length - 1 && peak_count < 20; i++) {
        if (buffer[i] > buffer[i-1] && 
            buffer[i] > buffer[i+1] && 
            buffer[i] > threshold) {
            peak_intervals[peak_count++] = i;
        }
    }
    
    if (peak_count < 3) return 0.0f;
    
    // Calculate interval regularity
    float mean_interval = 0;
    for (int i = 1; i < peak_count; i++) {
        mean_interval += (peak_intervals[i] - peak_intervals[i-1]);
    }
    mean_interval /= (peak_count - 1);
    
    float variance = 0;
    for (int i = 1; i < peak_count; i++) {
        float diff = (peak_intervals[i] - peak_intervals[i-1]) - mean_interval;
        variance += diff * diff;
    }
    variance /= (peak_count - 1);
    
    // Higher regularity = lower variance
    return 1.0f / (1.0f + variance);
}

// Calculate calories burned during eating (simplified)
float calculate_eating_calories(int duration_seconds) {
    // MET value for eating is about 1.5
    // Assuming 20kg dog
    float met_value = 1.5f;
    float weight_kg = 20.0f;
    
    return (met_value * 3.5f * weight_kg * duration_seconds) / (200.0f * 60.0f);
}

void detect_crunch_lap_sounds(float* audio_buffer, int length, float rms_level) {
    static time_t last_sound_time = 0;
    static int sound_pattern_count = 0;
    static bool drinking_in_progress = false;
    static bool eating_in_progress = false;
    
    // Only analyze if there's significant sound
    if (rms_level < 40.0f) {  // Quiet threshold
        sm.sensor_data.crunch_lap_sound = false;
        return;
    }
    
    // Perform frequency analysis (simplified - in real code, use FFT)
    float dominant_freq = estimate_dominant_frequency(audio_buffer, length);
    float pattern_regularity = calculate_pattern_regularity(audio_buffer, length);
    
    time_t current_time = time(NULL);
    
    // Detect CRUNCHING sound (eating food)
    if (dominant_freq >= CRUNCH_FREQUENCY_MIN && 
        dominant_freq <= CRUNCH_FREQUENCY_MAX &&
        pattern_regularity > 0.7f) {
        
        sm.sensor_data.crunch_lap_sound = true;
        sound_pattern_count++;
        
        if (!eating_in_progress) {
            eating_in_progress = true;
            ESP_LOGI(TAG, "Crunching sound detected - likely eating");
            
            // Reset drinking if eating starts
            drinking_in_progress = false;
        }
        
        // Update last detection time
        last_sound_time = current_time;
        
    } 
    // Detect LAPPING sound (drinking water)
    else if (dominant_freq >= LAP_FREQUENCY_MIN && 
             dominant_freq <= LAP_FREQUENCY_MAX &&
             pattern_regularity > 0.5f) {
        
        sm.sensor_data.crunch_lap_sound = true;
        sound_pattern_count++;
        
        if (!drinking_in_progress) {
            drinking_in_progress = true;
            ESP_LOGI(TAG, "Lapping sound detected - likely drinking");
            
            // Reset eating if drinking starts
            eating_in_progress = false;
        }
        
        // Update last detection time
        last_sound_time = current_time;
        
        // If we've detected enough consecutive lap sounds, update last_drink_time
        if (sound_pattern_count >= 3) {  // 3 consecutive detections
            sm.sensor_data.last_drink_time = current_time;
            ESP_LOGI(TAG, "Drinking confirmed - updated last_drink_time");
        }
    }
    else {
        sm.sensor_data.crunch_lap_sound = false;
        
        // Check if sound pattern has ended
        if (last_sound_time > 0 && 
            difftime(current_time, last_sound_time) > 2.0f) {
            
            // If we had an ongoing eating session that lasted long enough
            if (eating_in_progress && sound_pattern_count >= EATING_DURATION_SEC) {
                ESP_LOGI(TAG, "Eating session completed");
                sm.sensor_data.calorie_expenditure += calculate_eating_calories(sound_pattern_count);
            }
            
            // If we had an ongoing drinking session
            if (drinking_in_progress && sound_pattern_count >= DRINKING_DURATION_SEC) {
                ESP_LOGI(TAG, "Drinking session completed");
            }
            
            // Reset counters
            sound_pattern_count = 0;
            eating_in_progress = false;
            drinking_in_progress = false;
        }
    }
}

// Audio analysis for bark, panting, and crunch detection
void analyze_audio(float* audio_buffer, int length, float rms_level) {
    // Bark detection (based on loudness and frequency)
    if (rms_level > BARK_THRESHOLD_DB) {
        sm.sensor_data.bark_detected = true;
        ESP_LOGI(TAG, "Bark detected! Level: %.2f dB", rms_level);
    } else {
        sm.sensor_data.bark_detected = false;
    }
    
    // Simple frequency analysis for panting detection
    float dominant_freq = 0.0f;
    // In real implementation, you would do FFT here
    // For now, simulate frequency detection
    
    if (dominant_freq > 1.0f && dominant_freq < 4.0f) {
        sm.sensor_data.panting_detected = true;
        sm.sensor_data.respiration_rate = dominant_freq * 60.0f;  // Convert to breaths/min
    } else {
        sm.sensor_data.panting_detected = false;
    }
    
    // Crunch/lap sound detection (based on specific pattern)
	detect_crunch_lap_sounds(audio_buffer, length, rms_level);
    // This would require more advanced audio pattern recognition
}
