/*
 * hal_spi.c
 * HAL Layer – SPI bus + flash device init.
 */

#include "hal_spi.h"
#include "esp_log.h"

static const char *TAG = "HAL_SPI";

spi_device_handle_t hal_spi_flash = NULL;

void hal_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num     = HAL_SPI_MISO_GPIO,
        .mosi_io_num     = HAL_SPI_MOSI_GPIO,
        .sclk_io_num     = HAL_SPI_CLK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 32,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = HAL_SPI_CS_GPIO,
        .queue_size     = 1,
        .flags          = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &hal_spi_flash));

    ESP_LOGI(TAG, "SPI bus initialised");
}
