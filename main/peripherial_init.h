/*
 * peripherial_init.h
 *
 *  Created on: Feb 5, 2026
 *      Author: Dheeraj.Jain
 */

#ifndef MAIN_PERIPHERIAL_INIT_H_
#define MAIN_PERIPHERIAL_INIT_H_

#include"../components/common/config.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "../components/temperature/jirs40.h"
#include "../components/ext_flash/w25q64jv.h"

void i2c_init(void);
void gpio_init(void);
void spi_init(void);
void adc_init(void);
void adc_calibration_init(void);
void check_bno085(void);
void print_cpu_usage(void);
void print_ram_usage(void);

#endif /* MAIN_PERIPHERIAL_INIT_H_ */
