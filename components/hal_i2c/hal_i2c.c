/*
 * hal_i2c.c
 * HAL Layer – I2C master bus + device handle creation.
 */

#include "hal_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HAL_I2C";

i2c_master_bus_handle_t hal_i2c_bus     = NULL;
i2c_master_dev_handle_t hal_i2c_bno085  = NULL;
i2c_master_dev_handle_t hal_i2c_max30102 = NULL;

void hal_i2c_init(void)
{
    /* ── Bus ─────────────────────────────────────────────── */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port              = I2C_NUM_0,
        .sda_io_num            = HAL_I2C_SDA_GPIO,
        .scl_io_num            = HAL_I2C_SCL_GPIO,
        .clk_source            = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt     = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &hal_i2c_bus));

    /* Wait for BNO085 boot */
    vTaskDelay(pdMS_TO_TICKS(800));
    hal_i2c_scan();

    /* ── BNO085 device ───────────────────────────────────── */
    i2c_device_config_t bno_cfg = {
        .device_address = HAL_I2C_ADDR_BNO085,
        .scl_speed_hz   = HAL_I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(hal_i2c_bus, &bno_cfg, &hal_i2c_bno085));

    /* ── MAX30102 device ─────────────────────────────────── */
    i2c_device_config_t max_cfg = {
        .device_address = HAL_I2C_ADDR_MAX30102,
        .scl_speed_hz   = HAL_I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(hal_i2c_bus, &max_cfg, &hal_i2c_max30102));

    ESP_LOGI(TAG, "I2C bus and devices initialised");
}

void hal_i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(hal_i2c_bus, addr, 20) == ESP_OK) {
            if (addr == HAL_I2C_ADDR_BNO085)
                ESP_LOGI(TAG, "Found BNO085 @ 0x%02X", addr);
            else if (addr == HAL_I2C_ADDR_MAX30102)
                ESP_LOGI(TAG, "Found MAX30102 @ 0x%02X", addr);
            else
                ESP_LOGI(TAG, "Unknown device @ 0x%02X", addr);
        }
    }
}
