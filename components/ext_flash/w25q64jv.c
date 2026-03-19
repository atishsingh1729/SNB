/*
 * w25q64jv.c
 *
 *  Created on: Jan 6, 2026
 *      Author: Dheeraj.Jain
 */
#include "w25q64jv.h"

spi_device_handle_t flash;

#define TAG "EXT_FLASH"

void read_jedec_id(void)
{
    uint8_t tx[4] = {0x9F, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};

    spi_transaction_t t = {
        .length = 8 * sizeof(tx),   // TX = 4 bytes
        .tx_buffer = tx,
        .rx_buffer = rx,            // RX = 4 bytes (full duplex)
    };

    esp_err_t ret = spi_device_transmit(flash, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "JEDEC ID: %02X %02X %02X",
                 rx[1], rx[2], rx[3]);
    } else {
        ESP_LOGE(TAG, "SPI transmit failed");
    }
}

