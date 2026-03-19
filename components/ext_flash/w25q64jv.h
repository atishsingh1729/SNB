/*
 * w25q64jv.h
 *
 *  Created on: Jan 6, 2026
 *      Author: Dheeraj.Jain
 */

#ifndef COMPONENTS_EXT_FLASH_W25Q64JV_H_
#define COMPONENTS_EXT_FLASH_W25Q64JV_H_

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

extern spi_device_handle_t flash;

void read_jedec_id(void);


#endif /* COMPONENTS_EXT_FLASH_W25Q64JV_H_ */
