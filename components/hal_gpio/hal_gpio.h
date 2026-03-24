/*
 * hal_gpio.h
 * HAL Layer – GPIO config for LED, BNO085 RST/INT.
 */

#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

#include "driver/gpio.h"

#define HAL_GPIO_LED        GPIO_NUM_16
#define HAL_GPIO_BNO_RST    GPIO_NUM_38
#define HAL_GPIO_BNO_INT    GPIO_NUM_48

void hal_gpio_init(void);

#endif /* HAL_GPIO_H_ */
