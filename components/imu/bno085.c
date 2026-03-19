#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_timer.h"
#include "bno085.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../main/sensor_data_processing/signal_list.h"
#include <math.h>
#include "../../main/Feature_processing/feature_processing.h"
#include "../../components/common/config.h"
#include <esp_dsp.h>


s_all_raw_signals_t raw_signals;

static float acc_sum_x = 0, acc_sum_y = 0, acc_sum_z = 0;
static float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

static uint32_t imu_sample_count = 0;
static uint32_t last_imu_avg_time = 0;

// Sensor variables
float imu_variance = 0;                          // Variance of acceleration
bool gait_asymmetry = false;                     // Gait asymmetry flag
float range_of_motion = 0;                       // ROM percentage

// Internal variables for calculations
static float prev_accel_magnitude = 0;
static uint64_t last_step_time = 0;
static float step_times[10] = {0};               // Store last 10 step intervals
static int step_index = 0;
static float total_accel_variance = 0;
static int variance_samples = 0;
static float integrated_angle = 0;
static uint64_t last_rom_update = 0;
static uint64_t last_gyro_update = 0;

extern i2c_master_dev_handle_t bno_dev;

// ================== SHTP Write ===================
esp_err_t shtp_write(uint8_t *data, size_t len)
{
    return i2c_master_transmit(bno_dev, data, len, pdMS_TO_TICKS(1000));
}

// ================== SHTP Read ===================
esp_err_t shtp_read(uint8_t *buffer, size_t len)
{
    return i2c_master_receive(bno_dev, buffer, len, pdMS_TO_TICKS(1000));
}

// ================== Enable Accel + Gyro ===================
void enable_imu_feature(uint8_t sensor_id, uint32_t report_interval_us)
{
    uint8_t cmd[] = {
        21, 0,          // length = 21 (0x15), little endian
        2,              // channel
        1,     // sequence

        0xFD,sensor_id,           // Accelerometer

        0x00, 0x00, 0x00,   // reserved
        report_interval_us & 0xFF,
        (report_interval_us >> 8) & 0xFF,
        (report_interval_us >> 16) & 0xFF,
        (report_interval_us >> 24) & 0xFF,        
       // 0x60,0xEA,0x00,0x00,
		0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00
    };

    shtp_write(cmd, sizeof(cmd));
 }

void calculate_range_of_motion(void) {
    uint64_t current_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
    
    if (last_gyro_update > 0) {
        float dt = (current_time - last_gyro_update) / 1000.0f;  // Convert to seconds
        float angular_speed = sqrtf(sm.sensor_data.gyro_x * sm.sensor_data.gyro_x + 
        							sm.sensor_data.gyro_y * sm.sensor_data.gyro_y + 
        							sm.sensor_data.gyro_z * sm.sensor_data.gyro_z);
        integrated_angle += angular_speed * dt;
        
        // Reset integrated angle periodically (every 5 seconds)
        if (current_time - last_rom_update > 5000) {
            // Calculate ROM as percentage (normalized to 180 degrees max)
            sm.sensor_data.range_of_motion = fminf((integrated_angle / M_PI) * 100.0f, 100.0f);
            integrated_angle = 0;
            last_rom_update = current_time;
        }
    }
    
    last_gyro_update = current_time;
}


void calculate_variance(float ax, float ay, float az)
{
    static float mag_buffer[VAR_WINDOW_SIZE];
    static uint16_t var_index = 0;
    static uint16_t var_count = 0;

    float mag = sqrtf(ax*ax + ay*ay + az*az);

    // Store in circular buffer
    mag_buffer[var_index] = mag;
    var_index = (var_index + 1) % VAR_WINDOW_SIZE;

    if (var_count < VAR_WINDOW_SIZE)
        var_count++;

    // Only compute when buffer filled once
    if (var_count == VAR_WINDOW_SIZE)
    {
        float sum = 0.0f;
        float sum_sq = 0.0f;

        for (int i = 0; i < VAR_WINDOW_SIZE; i++)
        {
            sum += mag_buffer[i];
            sum_sq += mag_buffer[i] * mag_buffer[i];
        }

        float mean = sum / VAR_WINDOW_SIZE;
        sm.sensor_data.acc_variance =
            (sum_sq / VAR_WINDOW_SIZE) - (mean * mean);
    }
}
void parse_accel(uint8_t* p)
{
    int16_t x = p[4] | (p[5]<<8);
    int16_t y = p[6] | (p[7]<<8);
    int16_t z = p[8] | (p[9]<<8);

    sm.sensor_data.accel_x = x / 256.0f;
    sm.sensor_data.accel_y = y / 256.0f;
    sm.sensor_data.accel_z = z / 256.0f;

    calculate_variance(
        sm.sensor_data.accel_x,
        sm.sensor_data.accel_y,
        sm.sensor_data.accel_z
    );
	
#if DEBUG_LOGS
	printf("Accel X = %0.4f,--------------- Y = %0.4f,----------Z = %0.4f\n",	sm.sensor_data.accel_x,
	   sm.sensor_data.accel_y,
	   sm.sensor_data.accel_z);
#endif	
	
}

void parse_gyro(uint8_t* p)
{
    int16_t x = p[4] | (p[5]<<8);
    int16_t y = p[6] | (p[7]<<8);
    int16_t z = p[8] | (p[9]<<8);
	//printf("Gyro (raw hex): X: %04X, Y: %04X, Z: %04X\n", x, y, z);
	//printf("Gyro (raw int): X: %d Y: %d, Z: %d\n", x, y, z);
     
	/*These are in rads/s */	
	float X_rads = x / 512.0; //rads/s 
    float Y_rads = y / 512.0;
    float Z_rads = z / 512.0;
	
	 /*These are in deg/s*/
	 float X_deg = X_rads * (180.0f/M_PI);  
     float Y_deg = Y_rads * (180.0f/M_PI);
     float Z_deg = Z_rads * (180.0f/M_PI);
	 
	 
	 
	 sm.sensor_data.gyro_x = X_deg;
	 sm.sensor_data.gyro_y = Y_deg;
	 sm.sensor_data.gyro_z = Z_deg;

	 
#if DEBUG_LOGS	 
     printf("GYR (deg/s): %.4f %.4f %.4f\n",X_deg,Y_deg,Z_deg); 
#endif	 
}


void parse_linear_acce(uint8_t* p){
	int16_t x = p[4] | (p[5]<<8);
    int16_t y = p[6] | (p[7]<<8);
    int16_t z = p[8] | (p[9]<<8);
	
	float X = x / 256.0f;
	float Y = y / 256.0f; 
	float Z = z / 256.0f;
	
	
	sm.sensor_data.lin_accel_x = X;
	sm.sensor_data.lin_accel_y = Y;
	sm.sensor_data.lin_accel_z = Z;
	
#if  DEBUG__LOGS	 	
	printf("Linear Accel (float): X: %.4f ----------------------Y: %.4f,-------------------Z: %4f\n", X, Y, Z);
#endif	
}



void parse_id_stepcount(uint8_t* p)
{
    static uint32_t prev_steps = 0;
    static uint64_t window_start_ms = 0;

    uint32_t steps = p[8] | (p[9]<<8);
    uint64_t now_ms = esp_timer_get_time() / 1000;

    sm.sensor_data.total_steps = steps;

    if (window_start_ms == 0)
        window_start_ms = now_ms;

    float window_dt = (now_ms - window_start_ms) / 1000.0f;
    uint32_t delta_steps = steps - prev_steps;

    if (window_dt >= STEP_FREQ_UPDATE_WIN_SEC)//update every 2 seconds window
    {
        sm.sensor_data.step_frequency = delta_steps / window_dt;
        // Reset window
        window_start_ms = now_ms;
        prev_steps = steps;

		#if  DEBUG__LOGS	
			printf("StepCount(raw int):%"PRIu32" AND step frequency = %0.4f \n",  sm.sensor_data.total_steps, sm.sensor_data.step_frequency);
		#endif	
    }

}


void parse_game_rotation_vector(uint8_t* p)
{
    int16_t i = p[4]  | (p[5]  << 8);
    int16_t j = p[6]  | (p[7]  << 8);
    int16_t k = p[8]  | (p[9]  << 8);
    int16_t r = p[10] | (p[11] << 8);

    // Convert to float
    float q_i = i / 8192.0f;
    float q_j = j / 8192.0f;
    float q_k = k / 8192.0f;
    float q_r = r / 8192.0f;

    // ---------------------------
    // 1️⃣ Normalize quaternion
    // ---------------------------
    float norm = sqrtf(q_r*q_r + q_i*q_i + q_j*q_j + q_k*q_k);

    if (norm > 0.0f)
    {
        q_r /= norm;
        q_i /= norm;
        q_j /= norm;
        q_k /= norm;
    }

    // ---------------------------
    // 2️⃣ Roll
    // ---------------------------
    float roll = atan2f(
        2.0f * (q_r*q_i + q_j*q_k),
        1.0f - 2.0f * (q_i*q_i + q_j*q_j)
    );

    // ---------------------------
    // 3️⃣ Pitch (CLAMPED)
    // ---------------------------
    float sinp = 2.0f * (q_r*q_j - q_k*q_i);

    if (sinp >= 1.0f)
        sinp = 1.0f;
    else if (sinp <= -1.0f)
        sinp = -1.0f;

    float pitch = asinf(sinp);

    // ---------------------------
    // 4️⃣ Yaw
    // ---------------------------
    float yaw = atan2f(
        2.0f * (q_r*q_k + q_i*q_j),
        1.0f - 2.0f * (q_j*q_j + q_k*q_k)
    );

    // ---------------------------
    // 5️⃣ Convert to degrees
    // ---------------------------
    const float RAD_TO_DEG = 57.2957795f;

    float roll_deg  = roll  * RAD_TO_DEG;
    float pitch_deg = pitch * RAD_TO_DEG;
    float yaw_deg   = yaw   * RAD_TO_DEG;

#if DEBUG__LOGS
    printf("Roll: %.3f°, Yaw: %.3f°, Pitch: %.3f°\n",
           roll_deg, yaw_deg, pitch_deg);
#endif

    // Update structure
    sm.sensor_data.roll  = roll_deg;
    sm.sensor_data.pitch = pitch_deg;
    sm.sensor_data.yaw   = yaw_deg;
}


void parse_mag(uint8_t* p){
	int16_t x = p[4] | (p[5]<<8);
    int16_t y = p[6] | (p[7]<<8);
    int16_t z = p[8] | (p[9]<<8);
	
	float X = x / 16.0f;
	float Y = y / 16.0f; 
	float Z = z / 16.0f;
	
	sm.sensor_data.magneto_x = X;
	sm.sensor_data.magneto_y = Y;
	sm.sensor_data.magneto_z = Z;
	
#if DEBUG__LOGS
	printf("Magnetometer (float): X: %.4f Y: %.4f, Z: %4f\n", X, Y, Z);
#endif		
}


void parse_id_heart_rate(uint8_t* p){
	
	int16_t hr_rate_imu = p[4] | (p[5]<<8);
	printf("HEART RATE (int): %d BPM",hr_rate_imu);
}


void parse_id_gyro_rot(uint8_t* p)
{
	    /* ----------------------------------
	      Extract raw quaternion (Q14)
	       ---------------------------------- */
	    int16_t i = p[0] | (p[1] << 8);
	    int16_t j = p[2] | (p[3] << 8);
	    int16_t k = p[4] | (p[5] << 8);
	    int16_t r = p[6] | (p[7] << 8);

	    /* Convert to float */
	    float q_i = i / 16384.0f;
	    float q_j = j / 16384.0f;
	    float q_k = k / 16384.0f;
	    float q_r = r / 16384.0f;

	    /* ----------------------------------
	       Normalize quaternion
	       ---------------------------------- */
	    float norm = sqrtf(q_r*q_r + q_i*q_i + q_j*q_j + q_k*q_k);

	    if (norm > 0.0f)
	    {
	        q_r /= norm;
	        q_i /= norm;
	        q_j /= norm;
	        q_k /= norm;
	    }

	    /* ----------------------------------
	       Roll
	       ---------------------------------- */
	    float roll = atan2f(
	        2.0f * (q_r*q_i + q_j*q_k),
	        1.0f - 2.0f * (q_i*q_i + q_j*q_j)
	    );

	    /* ----------------------------------
	       Pitch (CLAMPED)
	       ---------------------------------- */
	    float sinp = 2.0f * (q_r*q_j - q_k*q_i);

	    if (sinp >= 1.0f)
	        sinp = 1.0f;
	    else if (sinp <= -1.0f)
	        sinp = -1.0f;

	    float pitch = asinf(sinp);

	    /* ----------------------------------
	       5️⃣ Yaw
	       ---------------------------------- */
	    float yaw = atan2f(
	        2.0f * (q_r*q_k + q_i*q_j),
	        1.0f - 2.0f * (q_j*q_j + q_k*q_k)
	    );

	    /* ----------------------------------
	       6️⃣ Convert to degrees
	       ---------------------------------- */
	    const float RAD_TO_DEG = 57.2957795f;

	    float roll_deg  = roll  * RAD_TO_DEG;
	    float pitch_deg = pitch * RAD_TO_DEG;
	    float yaw_deg   = yaw   * RAD_TO_DEG;

		printf("GIRV -> Roll: %.3f°, Yaw: %.3f°, Pitch: %.3f°\n",
		           roll_deg, yaw_deg, pitch_deg);

	#if DEBUG__LOGS
	    printf("GIRV -> Roll: %.3f°, Yaw: %.3f°, Pitch: %.3f°\n",
	           roll_deg, yaw_deg, pitch_deg);
	#endif

	    /* ----------------------------------
	       Extract angular velocity (Q10)
	       ---------------------------------- */
	    int16_t gx_raw = p[8]  | (p[9]  << 8);
	    int16_t gy_raw = p[10] | (p[11] << 8);
	    int16_t gz_raw = p[12] | (p[13] << 8);

	    float gx = gx_raw / 1024.0f;  // rad/s
	    float gy = gy_raw / 1024.0f;
	    float gz = gz_raw / 1024.0f;

	#if DEBUG__LOGS
	    printf("Gyro rad/s: X=%.3f Y=%.3f Z=%.3f\n", gx, gy, gz);
	#endif


}


/*
void analyze_fft(float *input)
{
    static float fft_buffer[FFT_SIZE * 2];
    static float magnitude[FFT_SIZE/2];

     Convert real input → complex buffer 
    for(int i = 0; i < FFT_SIZE; i++)
    {
        fft_buffer[2*i]   = input[i]; // real
        fft_buffer[2*i+1] = 0.0f;     // imag
    }

     Initialize FFT 
    dsps_fft2r_init_fc32(NULL, FFT_SIZE);

     Run FFT 
    dsps_fft2r_fc32(fft_buffer, FFT_SIZE);

     Bit reversal 
    dsps_bit_rev_fc32(fft_buffer, FFT_SIZE);

     Compute magnitude 
    for(int i = 0; i < FFT_SIZE/2; i++)
    {
        float real = fft_buffer[2*i];
        float imag = fft_buffer[2*i + 1];

        magnitude[i] = sqrtf(real*real + imag*imag);
    }

    printf("\n----- FFT Spectrum -----\n");

    for(int i = 1; i < FFT_SIZE/2; i++)
    {
        float freq = (i * SAMPLE_RATE) / FFT_SIZE;

        printf("Freq: %.3f Hz  Mag: %.5f\n", freq, magnitude[i]);
    }
}
*/




void analyze_fft(float *input)
{
    static float fft_buffer[FFT_SIZE * 2];
    static float magnitude[FFT_SIZE/2];

    float mean = 0;

   	/* Remove DC offset */
    for(int i = 0; i < FFT_SIZE; i++)
        mean += input[i];

    mean /= FFT_SIZE;

    /* Convert real input to complex */
    for(int i = 0; i < FFT_SIZE; i++)
    {
        fft_buffer[2*i]   = input[i] - mean;
        fft_buffer[2*i+1] = 0.0f;
    }

    /* Initialize FFT */
    dsps_fft2r_init_fc32(NULL, FFT_SIZE);

    /* Run FFT */
    dsps_fft2r_fc32(fft_buffer, FFT_SIZE);

    /* Bit reversal */
    dsps_bit_rev_fc32(fft_buffer, FFT_SIZE);

    /* Compute magnitude */
    for(int i = 0; i < FFT_SIZE/2; i++)
    {
        float real = fft_buffer[2*i];
        float imag = fft_buffer[2*i + 1];

        magnitude[i] = sqrtf(real*real + imag*imag);
    }

#if RESP_PRINT_SPECTRUM
    printf("\n----- Respiration Band FFT -----\n");
#endif

    float best_mag = 0;
    float best_freq = 0;
	
    /* Search respiration frequency band */
   
    for(int i = 1; i < FFT_SIZE/2; i++)
    {
        float freq = (i * SAMPLE_RATE) / FFT_SIZE;

        if(freq < RESP_FREQ_MIN_HZ || freq > RESP_FREQ_MAX_HZ)
            continue;

#if RESP_PRINT_SPECTRUM
        printf("Freq: %.3f Hz  Mag: %.5f\n", freq, magnitude[i]);
#endif

        if(magnitude[i] > best_mag && magnitude[i] > RESP_MAG_THRESHOLD)
        {
            best_mag = magnitude[i];
            best_freq = freq;
        }
    }

    /* Dominant respiration frequency */
    if(best_freq > 0)
    {
        float rr = best_freq * 60.0f;

        printf("\nDetected Respiration:\n");
        printf("Freq: %.3f Hz\n", best_freq);
        printf("Rate: %.2f BPM\n", rr);
    }
    else
    {
        printf("\nNo respiration peak detected\n");
    }
}




float estimate_respiration(StateMachine *sm)
{
    SensorData *s = &sm->sensor_data;

    /* ---------- static DSP state ---------- */

    static float lp = 0;                 // low pass
    static float prev_lp = 0;
    static float imu_env = 0;
    static float imu_prev = 0;
    static float imu_diff_prev = 0;

    static float ppg_env = 0;
    static float ppg_prev = 0;
    static float ppg_diff_prev = 0;

    static int64_t imu_last_peak = 0;
    static int64_t ppg_last_peak = 0;

    static float rr_imu = 0;
    static float rr_ppg = 0;

    static float rr_filtered = 0;

    int64_t now = esp_timer_get_time();

    /* if HR invalid, skip respiration */
    if(s->heart_rate == 0)
        return rr_filtered;

    /* ===================================================== */
    /*                  IMU RESPIRATION                      */
    /* ===================================================== */

    float ax = s->lin_accel_x;
    float ay = s->lin_accel_y;
    float az = s->lin_accel_z;

    /* magnitude squared (no sqrt for speed) */
    float acc = ax*ax + ay*ay + az*az;

    /* ignore large motion */
    float motion = fabsf(ax) + fabsf(ay) + fabsf(az);
    if(motion > 3.0f)
        return rr_filtered;

    /* low pass (~1 Hz) */
    lp += 0.05f * (acc - lp);

    /* high pass (~0.1 Hz) */
    float resp_signal = lp - prev_lp;
    prev_lp = lp;

    /* envelope detection */
    float vib = fabsf(resp_signal);
    imu_env += 0.02f * (vib - imu_env);

    float diff = imu_env - imu_prev;
    bool peak = (imu_diff_prev > 0 && diff <= 0);

    if (peak && imu_env > 0.0005f)
    {
        if (imu_last_peak != 0)
        {
            float interval = (now - imu_last_peak) / 1000000.0f;

            /* dog respiration range: ~15-200 BPM */
            if (interval > 0.3f && interval < 4.0f)
                rr_imu = 60.0f / interval;
        }

        imu_last_peak = now;
    }

    imu_diff_prev = diff;
    imu_prev = imu_env;

    /* PPG RESPIRATION */

    bool ppg_valid = (s->rawIR > 20000 && s->heart_rate > 30);

    if (ppg_valid)
    {
        float env = (float)s->rawIR;

        ppg_env += 0.02f * (env - ppg_env);

        float diff = ppg_env - ppg_prev;

        bool peak = (ppg_diff_prev > 0 && diff <= 0);

        if (peak)
        {
            if (ppg_last_peak != 0)
            {
                float interval = (now - ppg_last_peak) / 1000000.0f;

                if (interval > 0.3f && interval < 6.0f)
                    rr_ppg = 60.0f / interval;
            }

            ppg_last_peak = now;
        }

        ppg_diff_prev = diff;
        ppg_prev = ppg_env;
    }

    /*SENSOR FUSION */

    float rr_final;

    if (!ppg_valid)
        rr_final = rr_imu;
    else
        rr_final = 0.6f * rr_imu + 0.4f * rr_ppg;

    /* smoothing filter */
    if(rr_filtered == 0)
        rr_filtered = rr_final;

    rr_filtered += 0.1f * (rr_final - rr_filtered);

    s->respiration_rate = rr_filtered;

    return rr_filtered;
}


typedef struct {
    float x;
    float y;
    float z;
    int64_t timestamp;
} accel_data_t;


accel_data_t accel_buffer_glb;

uint8_t get_report_length(uint8_t report_id)
{
    switch (report_id)
    {
        case FEATURE_ID_ACCEL: return 10;
        case FEATURE_ID_GYRO: return 10;
        case FEATURE_ID_MAG: return 10;
        case SH2_LINEAR_ACCEL: return 10;
        case GAME_ROTATION_VECTOR: return 12;
        case FEATURE_ID_STEPCOUNT: return 12;
		case FEATURE_ID_HEART_RATE: return 6;
		case FEATURE_ID_GRYO_ROTA: return 14;
        default:   return 0;
    }
}

void parse_shtp_packet(uint8_t *p, uint16_t len)
{
    uint8_t value = 10;
    
	while(value < len){
		
		uint8_t report_id = p[value-1];
		uint8_t report_len = get_report_length(report_id);
/*		
        printf("\n------------------------------\n");
        printf("Report ID   : 0x%02X\n", report_id);*/
//        printf("Report Len  : %d\n", report_len);
        
        if (report_len == 0 || (value + report_len - 1) > len) {
            // bad or incomplete packet
            return;
        }
        
    	/* 🔍 Print RAW bytes of THIS report only */
       /* printf("Raw Data    : ");
        for (uint16_t i = 0; i < report_len; i++) {
            printf("%02X ", p[value + i - 1]);
        }
        printf("\n");*/
        
    	switch(report_id)
    	{
	        case FEATURE_ID_ACCEL:
	            parse_accel(&p[value-1]);
	            break;
	        case FEATURE_ID_GYRO:
	            parse_gyro(&p[value-1]);
	            break;
	        case FEATURE_ID_MAG:
	            parse_mag(&p[value-1]);
	            break;
	        case SH2_LINEAR_ACCEL:
	            parse_linear_acce(&p[value-1]);
	            break;
			case GAME_ROTATION_VECTOR:
				parse_game_rotation_vector(&p[value-1]);
				break;
			case FEATURE_ID_STEPCOUNT:
				parse_id_stepcount(&p[value-1]);
				break;
			case FEATURE_ID_HEART_RATE:
				parse_id_heart_rate(&p[value-1]);
				break;
			case FEATURE_ID_GRYO_ROTA:
				parse_id_gyro_rot(&p[value-1]);
				break;
	        default:
	            return;
    	}
    	value += report_len;
    	//printf("VALUE: %d\n",value);
    }
	
}

void process_imu_data(void)
{
    uint8_t header[SHTP_HEADER_LEN];
    uint8_t payload[MAX_PAYLOAD_LEN];
 
 //   printf("\n---- NEW SHTP READ ----\n");
 
    /* Read SHTP header */
    if (shtp_read(header, SHTP_HEADER_LEN) != ESP_OK) {
        //printf("Header read FAILED\n");
        return;
    }
 
    /* Print raw header */
/*    printf("Header: ");
    for (int i = 0; i < SHTP_HEADER_LEN; i++) {
        printf("%02X ", header[i]);
    }
    printf("\n");*/
 
    /* Extract packet length (LSB first) */
    uint16_t packet_len = (uint16_t)header[0] | ((uint16_t)header[1] << 8);

    /* Packet length includes header bytes */
    if (packet_len <= SHTP_HEADER_LEN) {
        printf("Invalid packet length\n");
        return;
    }

    uint16_t payload_len = packet_len ;//- SHTP_HEADER_LEN;
	payload_len &= 0x7FFF;
	
    /* Safety check */
    if (payload_len > MAX_PAYLOAD_LEN) {
        printf("Payload truncated! Requested: %u\n", payload_len);
        payload_len = MAX_PAYLOAD_LEN;
    }
 
  //  printf("Payload Length: %u\n", payload_len);
 
    /* Read payload */
    if (shtp_read(payload, payload_len) != ESP_OK) {
        printf("Payload read FAILED\n");
        return;
    }
 
    /* Print payload hex */
/*    printf("Payload: ");
    for (int i = 0; i < payload_len; i++) {
        printf("%02X ", payload[i]);
        if ((i + 1) % 16 == 0)
            printf("\n         ");
    }
    printf("\n");
 */
    /* Parse packet */
    parse_shtp_packet(payload, payload_len);
}
