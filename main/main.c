#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include"../../components/common/config.h"
#include "driver/gpio.h"

//-------------ESPRESSIF INCLUDE FILES-----------------
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
//#include "driver/uart.h"
#include "esp_log.h"
#include "cJSON.h"
#include "peripherial_init.h"
#include <sys/time.h>
//-------------SENSORS INCLUDE FILES----------------
#include "../components/imu/bno085.h"
#include "../components/hr/max30102.h"
#include "../components/ext_flash/w25q64jv.h"
#include "../components/temperature/jirs40.h"
#include "../components/time_manager/systemTime.h"
//-------------PROJECT INCLUDE FILES-----------------
#include "Feature_processing/feature_processing.h"
#include "../components/ble/ble.h"
#include "../components/WiFi/wifi.h"
#include "esp_wifi.h"
#include "struct_2_json.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/idf_additions.h"
#include "../components/common/datatypes.h"

static const char *TAG = "MAIN";
/*static float batt_percent_buffer[BATT_AVG_SAMPLES] = {0};
static uint8_t batt_index = 0;
static uint8_t batt_count = 0;
float batt_voltage_global =0;
float Batt_percentage = 0;*/
/*float global_raw_body,global_raw_amb;*/


ADC_Variables adc1_val;
systemTime time_val;


s_json_packet_t pkt;
extern i2c_master_dev_handle_t max30102_dev;
extern i2c_master_bus_handle_t i2c_bus;
int32_t packet_count_global;
int32_t	raw_ir_global = 0;

// Data structures
typedef struct {
    float heart_rate;
    float confidence;
    uint8_t valid;
    int peak_count;
    float interbeat_interval_ms;
} heart_rate_result_t;

uint32_t hr_globle=0;
uint32_t ir_rowData=0;
uint8_t state_dog=0;

// Global variables
static SemaphoreHandle_t i2c_mutex;


uint8_t global_ble_buff[1024];//BlE global buffer



extern adc_cali_handle_t adc_cali_temp;
extern adc_cali_handle_t adc_cali_batt;





void send_ble_data(void *arg){
	while(1){
	handle_json_packet(&sm, &pkt);

    cJSON *j = create_json_packet(&pkt);
    if (!j) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
    }

    char *json_str = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);

    if (json_str) {
#if DEBUG_LOGS		
        ESP_LOGI("JSON", "%s", json_str);
#endif		
        send_message_all((uint8_t *)json_str, strlen(json_str));
		memcpy(global_ble_buff, json_str,strlen(json_str));

        free(json_str);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second update
    
	}
}

void send_wifi_data(void *arg){
	while(1){
	// Wi-Fi complete process code
	wifi_hello();
	vTaskDelay(pdMS_TO_TICKS(1000));
	}
}




void cpu_status(void *args){
	
	while(1){
		print_cpu_usage();
		print_ram_usage();
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

//----------------------
void led_blink(void *arg){
	while(1){
		gpio_set_level(LED_GPIO, 1);  // LED ON
	    vTaskDelay(pdMS_TO_TICKS(500));
	
	    gpio_set_level(LED_GPIO, 0);  // LED OFF
	   	vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void get_temp(void *arg)
{
    float vout_body, vout_amb;
    float temp_body, temp_amb;
	float Batt_volt, div_volt;

    while (1) {

        /* BODY */
        vout_body = adc_read(ADC_CHANNEL_BODY,adc_cali_temp);
        float r_body = ntc_voltage_to_resistance(vout_body);
        temp_body = ntc_resistance_to_temp(r_body);
        temp_body = ntc_apply_calibration(temp_body);
		adc1_val.raw_volt_body = vout_body;
		adc1_val.body_temp = temp_body;

        /* AMBIENT */
        vout_amb = adc_read(ADC_CHANNEL_AMB,adc_cali_temp);
        float r_amb = ntc_voltage_to_resistance(vout_amb);
        temp_amb = ntc_resistance_to_temp(r_amb);
        temp_amb = ntc_apply_calibration(temp_amb);
		adc1_val.raw_volt_amb = vout_amb;
		adc1_val.amb_temp = temp_amb;
		
		/* Battery */
		div_volt = adc_read(ADC_CHANNEL_BATT, adc_cali_batt) * ADC_BATT_GAIN;
		Batt_volt = div_volt * 3.57f;
		adc1_val.raw_volt_battery = Batt_volt;
		float instant_percent = convert_to_percentage(Batt_volt);

		/* Store in rolling buffer */
		adc1_val.batt_percent_buffer[adc1_val.batt_index] = instant_percent;
		
		adc1_val.batt_index = (adc1_val.batt_index + 1) % BATT_AVG_SAMPLES;
		
		if (adc1_val.batt_count < BATT_AVG_SAMPLES)
		    adc1_val.batt_count++;

		/* Compute average */
		float sum = 0;
		for (int i = 0; i < adc1_val.batt_count; i++)
		    sum += adc1_val.batt_percent_buffer[i];

		adc1_val.Batt_percentage = sum / adc1_val.batt_count;		
#if DEBUG_LOGS		
        ESP_LOGW("TEMP ","Vout Body = %.3f V | Temp Body = %.2f C\n", vout_body, temp_body);
        ESP_LOGW("TEMP ","Vout AMB  = %.3f V | Temp Amb  = %.2f C\n\n", vout_amb, temp_amb);
		ESP_LOGW("BATT ","Divider Voltage = %0.4f , Battery Voltage = %0.4f , Battery Percent = %0.1f\n", div_volt,Batt_volt,adc1_val.Batt_percentage);
#endif
		sm.sensor_data.temperature = temp_amb;
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}


void i2c_task(void *arg)
{
    uint8_t fifo[6];
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI("I2C TASK", "Unified sensor task started (HR master @100Hz)");
    while (1)
    {

       
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

       uint8_t wr, rd;

        max30102_read_reg(REG_FIFO_WR_PTR, &wr);
        max30102_read_reg(REG_FIFO_RD_PTR, &rd);

        uint8_t samples = (wr - rd) & 0x1F;


		if (samples > 0)
		{
		    if (max30102_read_fifo(fifo, 6) == ESP_OK)
		    {
		        uint32_t ir = ((fifo[0] & 0x03) << 16) |
		                      (fifo[1] << 8) |
		                      fifo[2];

		        ir &= 0x03FFFF;

		        sm.sensor_data.rawIR = ir;
		        hr_process_sample(ir);
		    }
		}

		/* IMU PROCESSING (TIME-SLICED) */

		int64_t imu_start = esp_timer_get_time();

		while (gpio_get_level(BNO085_INT_PIN) == 0)
		{
		    process_imu_data();

		    if ((esp_timer_get_time() - imu_start) > 3000)  // 3 ms max
		        break;
		}

        process_state_transitions(&sm);
        state_dog = sm.current_state;
#if DEBUG_LOGS       
		 show_state(&sm);
#endif
	  }
}

void monitor_task(void *args)
{
    static float accel_buffer[FFT_SIZE];
    static int index = 0;

    float accel_mag;

    while(1)
    {
        /* Acceleration magnitude */
        accel_mag = sqrtf(
            sm.sensor_data.lin_accel_x * sm.sensor_data.lin_accel_x +
            sm.sensor_data.lin_accel_y * sm.sensor_data.lin_accel_y +
            sm.sensor_data.lin_accel_z * sm.sensor_data.lin_accel_z
        );

        accel_buffer[index++] = accel_mag;

        /* When buffer fills, analyze */
        if(index >= FFT_SIZE)
        {
            analyze_fft(accel_buffer);

            /* Shift buffer for overlap (75% overlap) */
            int shift = FFT_SIZE / 4;

            for(int i = 0; i < FFT_SIZE - shift; i++)
            {
                accel_buffer[i] = accel_buffer[i + shift];
            }

            index = FFT_SIZE - shift;
        }

        /* 100 Hz sampling */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/* ------------------------------------------------------------------ */
/*  app_main                                                            */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create i2c_mutex – aborting");
        return;
    }

    /* ── Peripheral init (single-threaded, no tasks running yet) ── */
    gpio_init();
    i2c_init();
    ble_init();
    spi_init();
    adc_init();
    adc_init_calibration();

    ESP_LOGI(TAG, "Reading JEDEC ID...");
    read_jedec_id();

 //    Small settle delay for power rails 
    vTaskDelay(pdMS_TO_TICKS(1000));

   // BNO085 init sequence
    ESP_LOGI(TAG, "Resetting BNO085...");
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	vTaskDelay(pdMS_TO_TICKS(200));
	//ESP_LOGI("BNOINIT", "Enabling features...");

    enable_imu_feature(FEATURE_ID_ACCEL,        10000);
    enable_imu_feature(FEATURE_ID_GYRO,         10000);
    enable_imu_feature(SH2_LINEAR_ACCEL,        10000);
    enable_imu_feature(FEATURE_ID_MAG,          10000);
    enable_imu_feature(FEATURE_ID_STEPCOUNT,    10000);
	enable_imu_feature(GAME_ROTATION_VECTOR,	10000);
	enable_imu_feature(FEATURE_ID_GRYO_ROTA,	10000);

    xSemaphoreGive(i2c_mutex);
   // ESP_LOGI(TAG, "BNO085 fully initialized");

    /* ── MAX30102 init ── */
    vTaskDelay(pdMS_TO_TICKS(50));
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    max30102_init(max30102_dev);
    xSemaphoreGive(i2c_mutex);
    vTaskDelay(pdMS_TO_TICKS(50));
	
	/*Set epoch Time */
	//get_time_from_cloud();	
	time_val.cloud_epoch_time = 1772185467;
	update_system_time(time_val.cloud_epoch_time);
	/*Initialize state machine */
	initialize_state_machine(&sm);
	
	
    /* ── Start tasks ── */
    xTaskCreate(led_blink,          "led_blink",    1024, NULL, 5,  NULL);
 	xTaskCreate(i2c_task, "i2c_sensors",8192, NULL,10, NULL);
    xTaskCreate(get_temp,       "temperature",  4096, NULL, 5,  NULL);
    xTaskCreate(send_ble_data,  "ble_tx",       4096, NULL, 5,  NULL);
    xTaskCreate(send_wifi_data, "wifi_tx",      4096, NULL, 5,  NULL);
	xTaskCreate(monitor_task, "monitor_task",      8192, NULL, 5,  NULL);
	//xTaskCreate(cpu_status,     "cpu_stats",    4096, NULL, 1,  NULL); 
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

