/*
 * hal_spi.h
 * HAL Layer – SPI bus abstraction (used by W25Q64JV flash).
 */

#ifndef HAL_SPI_H_
#define HAL_SPI_H_

#include "driver/spi_master.h"
#include "driver/gpio.h"

/* ── Pin config ──────────────────────────────────────────────── */
#define HAL_SPI_MISO_GPIO   GPIO_NUM_11
#define HAL_SPI_MOSI_GPIO   GPIO_NUM_13
#define HAL_SPI_CLK_GPIO    GPIO_NUM_12
#define HAL_SPI_CS_GPIO     GPIO_NUM_10
#define HAL_SPI_WP_GPIO     GPIO_NUM_14
#define HAL_SPI_HOLD_GPIO   GPIO_NUM_21

/* ── Public flash device handle ──────────────────────────────── */
extern spi_device_handle_t hal_spi_flash;

/* ── API ─────────────────────────────────────────────────────── */
void hal_spi_init(void);

#endif /* HAL_SPI_H_ */
