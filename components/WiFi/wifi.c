/*
 * wifi.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Sanjay.Chauhan
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_rtc_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include "wifi.h"
#include "../../main/Feature_processing/feature_processing.h"
#include "../../components/common/datatypes.h"
/* ================= CONFIG ================= */

#define WIFI_SSID     "ksplaim"
#define WIFI_PASS     "12345678"

#define UDP_TX_INTERVAL_MS 1000
#define BLE_STR_SIZE	   1024
/* ================= TAG ================= */

static const char *TAG = "wifi_udp";

/* ================= GLOBAL IP INFO ================= */

static esp_netif_ip_info_t g_ip_info;
static bool g_ip_ready = false;
//bpm_global
/* ================= EXTERNAL DATA ================= */

extern uint32_t bpm_global;
extern uint8_t state_dog;
extern int32_t raw_ir_global;
int count_itration = 0;
extern uint32_t ir_rowData;
extern float ac_dc_ratio_glb;
extern int sqi_bad_cnt_glb;
extern uint8_t global_ble_buff[1024];
extern float global_raw_body,global_raw_amb;
extern ADC_Variables adc1_val;
uint32_t packetCounter_5005 = 0;
/* ================= UDP SERVER CONFIG ================= */

typedef struct {
    int port;
    const char *name;
} udp_server_cfg_t;


/* ================= UDP TASK ================= */


typedef struct __attribute__((packed)){
    uint64_t RTC_clk;
    uint16_t Pyload_count;
	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;
	float Mx;
	float My;
	float Mz;
} IMU_senData_t;

IMU_senData_t IMU_senData;

typedef struct {
    uint64_t RTC_clk;
    uint32_t Pyload_count;
	uint32_t Hr_rw;
} HR_senData_t;

HR_senData_t HR_senData;

typedef struct __attribute__((packed)){
    uint64_t RTC_clk;
    uint16_t Pyload_count;
	float adc1;
	float adc2;
	float adc3;
} ADC_senData_t;

ADC_senData_t ADC_senData;



const char* get_state_name(StateType state)
{
    switch(state)
    {
        case STATE_RESTING_IDLE: return "RESTING_IDLE";
        case STATE_WALKING: return "WALKING";
        case STATE_RUNNING_PLAYING: return "RUNNING_PLAYING";
        case STATE_SLEEPING: return "SLEEPING";
        case STATE_BARKING: return "BARKING";
        case STATE_TREMBLING_SHAKING: return "TREMBLING_SHAKING";
        case STATE_EPILEPSY: return "EPILEPSY";
        case STATE_POST_SEIZURE: return "POST_SEIZURE";
        case STATE_ARTHRITIS: return "ARTHRITIS";
        case STATE_PAIN_INJURY: return "PAIN_INJURY";
        case STATE_FATIGUE_ALERT: return "FATIGUE_ALERT";
        case STATE_STRESS_ANXIETY: return "STRESS_ANXIETY";

        case STATE_EATING_DRINKING_DETECTING: return "EATING_DRINKING_DETECTING";
        case STATE_EATING_DRINKING_ACTIVE: return "EATING_DRINKING_ACTIVE";
        case STATE_EATING_DRINKING_COMPLETE: return "EATING_DRINKING_COMPLETE";

        case STATE_LICKING_SCRATCHING_GROOMING: return "LICKING_SCRATCHING_GROOMING";
        case STATE_LICKING_SCRATCHING_EXCESSIVE: return "LICKING_SCRATCHING_EXCESSIVE";
        case STATE_LICKING_SCRATCHING_MEDICAL_ALERT: return "LICKING_SCRATCHING_MEDICAL_ALERT";

		case STATE_NO_DEHYDRATION: return "STATE_NO_DEHYDRATION";
        case STATE_DEHYDRATION_DETECTED: return "DEHYDRATION_DETECTED";
        case STATE_DEHYDRATION_SEVERE: return "DEHYDRATION_SEVERE";
        case STATE_DEHYDRATION_CRITICAL: return "DEHYDRATION_CRITICAL";

        case STATE_RESPIRATION_NORMAL: return "RESPIRATION_NORMAL";
        case STATE_RESPIRATION_ELEVATED: return "RESPIRATION_ELEVATED";
        case STATE_TEMPERATURE_NORMAL: return "TEMPERATURE_NORMAL";
        case STATE_TEMPERATURE_FEVER: return "TEMPERATURE_FEVER";
        case STATE_TEMPERATURE_HYPOTHERMIA: return "TEMPERATURE_HYPOTHERMIA";
        case STATE_HRV_STABLE: return "HRV_STABLE";
        case STATE_HRV_STRESSED: return "HRV_STRESSED";
		case STATE_HRV_UNKNOW: return "HRV_UNREADABLE";

		case STATE_HR_UNKNOW: return "STATE_HR_UNKNOW";
		case STATE_HR_STABLE: return "STATE_HR_STABLE";
		case STATE_HR_LOW: return "STATE_HR_LOW";		
		case STATE_HR_HIGH: return "STATE_HR_HIGH";
		
		
		
        case STATE_UNKNOWN: return "UNKNOWN";

        default: return "INVALID_STATE";
    }
}



static void udp_tx_task(void *pvParameters)
{
    udp_server_cfg_t *cfg = (udp_server_cfg_t *)pvParameters;
    char tx_buffer[256] = {0};
	char tx_buffer_ble[BLE_STR_SIZE] = {0};
	//uint8_t *p = (uint8_t *)&IMU_senData;

    struct sockaddr_in dest_addr = {0};

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket create failed for %s", cfg->name);
        vTaskDelete(NULL);
        return;
    }

    /* Enable broadcast */
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(cfg->port);
    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255");

    ESP_LOGI(TAG, "%s transmitting on PORT %d (Broadcast)",
             cfg->name, cfg->port);

    while (1) {

        switch (cfg->port) {

        case 5000:
		
		 IMU_senData.RTC_clk = esp_rtc_get_time_us();;
		 IMU_senData.Pyload_count++;
		 IMU_senData.Ax = sm.sensor_data.accel_x;
		 IMU_senData.Ay = sm.sensor_data.accel_y;
		 IMU_senData.Az = sm.sensor_data.accel_z;
		 IMU_senData.Gx = sm.sensor_data.gyro_x;
		 IMU_senData.Gy = sm.sensor_data.gyro_y;
		 IMU_senData.Gz = sm.sensor_data.gyro_z;
		 IMU_senData.Mx = sm.sensor_data.magneto_x;
		 IMU_senData.My = sm.sensor_data.magneto_y;
		 IMU_senData.Mz = sm.sensor_data.magneto_z;
		 
		 
		 sendto(sock,
		        &IMU_senData,
		        sizeof(IMU_senData),
		        0,
		        (struct sockaddr *)&dest_addr,
		        sizeof(dest_addr));
				
			break;
			
			
		case 5001: 
			HR_senData.RTC_clk = esp_rtc_get_time_us();
			HR_senData.Pyload_count++;
			HR_senData.Hr_rw = raw_ir_global;
		
			sendto(sock,
		       &HR_senData,
		       sizeof(HR_senData),
		       0,
		       (struct sockaddr *)&dest_addr,
		       sizeof(dest_addr));
			
		break;
		
		case 5002:
		
			ADC_senData.RTC_clk = esp_rtc_get_time_us();
			ADC_senData.Pyload_count++;
			ADC_senData.adc1 = adc1_val.raw_volt_body;
			ADC_senData.adc2 = adc1_val.raw_volt_amb;
			ADC_senData.adc3 = adc1_val.raw_volt_battery;
				
			sendto(sock,
		       &ADC_senData,
		       sizeof(ADC_senData),
		       0,
		       (struct sockaddr *)&dest_addr,
		       sizeof(dest_addr));
		
			break;
			
		case 5003:	
			
			break;
			
        case 5004:
		//calculated step freq variance 
            snprintf(tx_buffer, sizeof(tx_buffer),
                     "MOTION, Totalsteps = %"PRIu32" , step_frequency = %.2f , variance = %.2f ",
                     sm.sensor_data.total_steps,sm.sensor_data.step_frequency,sm.sensor_data.acc_variance);
					 sendto(sock,
					               tx_buffer,
					               strlen(tx_buffer),
					               0,
					               (struct sockaddr *)&dest_addr,
					               sizeof(dest_addr));
            break;

        case 5005:
		//time and bpm 
            snprintf(tx_buffer, sizeof(tx_buffer),
                     "Time_us=%llu,Packet_count = %lu, HR=%lu, HRV = %0.2f, Resp Rate = %0.2f,IR=%lu",
                     esp_rtc_get_time_us(),
					 (unsigned long)packetCounter_5005++,
					 (unsigned long)sm.sensor_data.heart_rate,
					 sm.sensor_data.hrv,
					 sm.sensor_data.respiration_rate,
                     (unsigned long)sm.sensor_data.rawIR);
					 sendto(sock,
					               tx_buffer,
					               strlen(tx_buffer),
					               0,
					               (struct sockaddr *)&dest_addr,
					               sizeof(dest_addr));
            break;

        
        case 5006:
		//ble json 
            snprintf(tx_buffer_ble, sizeof(tx_buffer_ble),
                     "%s",
                    global_ble_buff);
					 sendto(sock,
					               tx_buffer_ble,
					               strlen(tx_buffer_ble),
					               0,
					               (struct sockaddr *)&dest_addr,
					               sizeof(dest_addr));
            break;


		
			case 5007:
			{
			    const char* main_state = get_state_name(sm.current_state);
			    const char* temp_state = get_state_name(sm.sub_states.temperature);
			    const char* resp_state = get_state_name(sm.sub_states.respiration);
			    const char* hrv_state  = get_state_name(sm.sub_states.hrv);
				const char* hr_state  = get_state_name(sm.sub_states.hr);
			    
				snprintf(tx_buffer, sizeof(tx_buffer),"MAIN=%s,""TEMP=%s,""RESP=%s,""HRV=%s,""HR=%s,""Battery=%.2f%%,""BodyTemp=%.2fC,""AmbTemp=%.2fC,""BattVolt=%.2fV",
			        main_state,
			        temp_state,
			        resp_state,
			        hrv_state,
					hr_state,
			        adc1_val.Batt_percentage,
			        adc1_val.body_temp,
			        adc1_val.amb_temp,
			        adc1_val.raw_volt_battery);

			    sendto(sock,
			           tx_buffer,
			           strlen(tx_buffer),
			           0,
			           (struct sockaddr *)&dest_addr,
			           sizeof(dest_addr));
			}
			break;			
			
			case 5008:
			    snprintf(tx_buffer, sizeof(tx_buffer),
			             "ORIENTATION IN DEGREES,  Roll = %0.3f ,  Yaw = %0.3f ,  Pitch = %0.3f",
			             sm.sensor_data.roll,sm.sensor_data.yaw,sm.sensor_data.pitch);
						 
						 sendto(sock,
						               tx_buffer,
						               strlen(tx_buffer),
						               0,
						               (struct sockaddr *)&dest_addr,
						               sizeof(dest_addr));
			    break;


        default:
            snprintf(tx_buffer, sizeof(tx_buffer), "UNKNOWN");
			sendto(sock,
			              tx_buffer,
			              strlen(tx_buffer),
			              0,
			              (struct sockaddr *)&dest_addr,
			              sizeof(dest_addr));
            break;
        }

       

        vTaskDelay(pdMS_TO_TICKS(UDP_TX_INTERVAL_MS));
    }
}

/* ================= START UDP ================= */

static void start_udp_servers(void)
{
	uint8_t totaltask = 9; 
    static udp_server_cfg_t udp_cfg[] = {
        {5000, "IMU-5000"},
        {5001, "HR-5001"},
        {5002, "TIME-5002"},
        {5003, "STATE-5003"},
        {5004, "IMU-5004"},
        {5005, "HR-5005"},
        {5006, "TIME-5006"},
        {5007, "STATE-5007"},
		{5008, "ORIENTATION"},
    };

    for (int i = 0; i < totaltask; i++) { //8
        xTaskCreate(
            udp_tx_task,
            udp_cfg[i].name,
            4096,
            &udp_cfg[i],
            5,
            NULL
        );
    }
}

/* ================= WIFI EVENTS ================= */

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi started, connecting...");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        g_ip_info = event->ip_info;
        g_ip_ready = true;

        ESP_LOGI(TAG, "==============================");
        ESP_LOGI(TAG, "ESP32 NETWORK READY");
        ESP_LOGI(TAG, "IP      : " IPSTR, IP2STR(&g_ip_info.ip));
        ESP_LOGI(TAG, "NETMASK : " IPSTR, IP2STR(&g_ip_info.netmask));
        ESP_LOGI(TAG, "GATEWAY : " IPSTR, IP2STR(&g_ip_info.gw));
        ESP_LOGI(TAG, "==============================");

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

        start_udp_servers();
    }
}

/* ================= WIFI INIT ================= */

void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA started, waiting for DHCP IP...");
}

/* ================= PUBLIC API ================= */

void wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    wifi_init_sta();
}

void wifi_hello(void)
{
    static bool wifi_started = false;

    if (!wifi_started) {
        wifi_init();
        ESP_LOGI(TAG, "WiFi initialized first time");
        wifi_started = true;
    } else {
        //ESP_LOGI(TAG, "WiFi already running");
    }
}
