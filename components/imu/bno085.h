#ifndef BNO085_H
#define BNO085_H
 
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "../../main/Feature_processing/feature_processing.h"


#define MAX_PAYLOAD_LEN 		512
#define FEATURE_ID_ACCEL   		0x01
#define FEATURE_ID_GYRO        	0x02
#define FEATURE_ID_MAG			0x03
#define SH2_LINEAR_ACCEL     	0x04
#define	GAME_ROTATION_VECTOR	0x08
#define	FEATURE_ID_STEPCOUNT	0x11
#define FEATURE_ID_HEART_RATE	0x23
#define FEATURE_ID_GRYO_ROTA	0x2A
#define SHTP_HEADER_LEN 		4
			

//Respiration
#define FFT_SIZE        1024
#define SAMPLE_RATE     100.0f   // Hz 

/* ===== Respiration FFT tuning ===== */
#define RESP_FREQ_MIN_HZ      0.15f
#define RESP_FREQ_MAX_HZ      0.80f

#define RESP_MAG_THRESHOLD    0.01f

#define RESP_PRINT_SPECTRUM   1
#define RESP_SHOW_WAVE        0

/* ===== Respiration RSA tuning ===== */
#define RSA_WINDOW_SIZE        64
#define RSA_MIN_BREATH_HZ      0.15f
#define RSA_MAX_BREATH_HZ      0.80f
#define RSA_FILTER_ALPHA       0.2f


void process_imu_data(void);

esp_err_t shtp_write(uint8_t *data, size_t len);

esp_err_t shtp_read(uint8_t *buffer, size_t len);

void bno085_process_loop(i2c_master_dev_handle_t dev);

void enable_imu_feature(uint8_t sensor_id, uint32_t report_interval_us);

void parse_accel(uint8_t* p);

void parse_gyro(uint8_t* p);

void parse_shtp_packet(uint8_t *p, uint16_t len);

float estimate_respiration(StateMachine *sm);

void analyze_fft(float *accel_data);

float estimate_respiration_rsa(float heart_rate);

#endif // BNO085_H
