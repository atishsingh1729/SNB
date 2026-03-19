/*
 * peripherial_init.c
 *
 *  Created on: Feb 5, 2026
 *      Author: Dheeraj.Jain
 */

#include "peripherial_init.h"

i2c_master_bus_handle_t i2c_bus;
i2c_master_dev_handle_t bno_dev;
i2c_master_dev_handle_t max30102_dev;

extern adc_oneshot_unit_handle_t adc_handle;
extern adc_cali_handle_t adc_cali_handle;

void i2c_scan_bus(i2c_master_bus_handle_t bus)
{
	//remove this and scan only address of device
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t err = i2c_master_probe(bus, addr, 20);
		if (err == ESP_OK) {
            //printf("I2C device found at 0x%02X\n", addr);
			if(addr == 0x4B || addr == 0x4A){
				printf("I2C bus ready for BNO085\n");
			}
			else if(addr == 0x57){
				printf("I2C bus ready for MAX30102\n");
			}
        }
		else{
			//printf("Address Not found\n");
		}
    }	
}



void print_ram_usage(void)
{
	static const char *TAG_STATS = "RAM";
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap  = esp_get_minimum_free_heap_size();

    uint32_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t internal_min  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
	

	ESP_LOGI(TAG_STATS, "========== RAM USAGE ==========");
	ESP_LOGI(TAG_STATS, "Free Heap       : %" PRIi32 " bytes", free_heap);
	ESP_LOGI(TAG_STATS, "Min Free Heap   : %" PRIi32 " bytes", min_heap);
	ESP_LOGI(TAG_STATS, "Internal Free   : %" PRIi32 " bytes", internal_free);
	ESP_LOGI(TAG_STATS, "Internal Min    : %" PRIi32 " bytes", internal_min);

}

void print_cpu_usage(void)
{
    static char pcStatsBuffer[STATS_BUFFER_SIZE];

    vTaskDelay(1);  // <-- IMPORTANT

    memset(pcStatsBuffer, 0, STATS_BUFFER_SIZE);
    vTaskGetRunTimeStats(pcStatsBuffer);

    vTaskDelay(1);  // <-- IMPORTANT

    ESP_LOGI("CPU", "\n%s", pcStatsBuffer);
}


void check_bno085(void){
	uint8_t dummy = 0x00;

	esp_err_t err = i2c_master_transmit(
	    bno_dev,
	    &dummy,
	    1,
	    pdMS_TO_TICKS(100)
	);

	ESP_LOGI("TEST", "I2C probe result: %s", esp_err_to_name(err));

}

void i2c_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
 
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &i2c_bus));
    
	vTaskDelay(pdMS_TO_TICKS(800)); //BNO wake up time  
    i2c_scan_bus(i2c_bus);
 //remove this and put this on peripherial init file
    i2c_device_config_t devcfg = {
        .device_address = BNO085_ADDR,
        .scl_speed_hz   = I2C_SPEED_HZ
    };
 
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &devcfg, &bno_dev));
 
    /* -------- MAX30102 device -------- */
    i2c_device_config_t max_cfg = {
        .device_address = MAX30102_ADDR,
        .scl_speed_hz   = I2C_SPEED_HZ // MAX30102 prefers 100kHz
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &max_cfg, &max30102_dev));
}
 
//================= GPIO INIT================================
void gpio_init(void)
{
    gpio_config_t io_conf = {0};

    /* ================= LED GPIO ================= */
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(LED_GPIO, 0);   // LED OFF


    /* ================= BNO085 RESET PIN ================= */
    io_conf.pin_bit_mask = (1ULL << BNO085_RST_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;   
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(BNO085_RST_PIN, 1);  // Keep out of reset


    /* ================= BNO085 INT PIN ================= */
    io_conf.pin_bit_mask = (1ULL << BNO085_INT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;   // REQUIRED
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;     // We will poll it
    gpio_config(&io_conf);
}



//================= SPI INIT===================================

void spi_init(void){
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,   // 1 MHz safe
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
		.flags = 0,   // <<< IMPORTAN
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &flash));

}

