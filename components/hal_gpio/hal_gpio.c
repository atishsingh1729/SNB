/*
 * hal_gpio.c
 * HAL Layer – GPIO init for LED, BNO085 RST/INT.
 */

#include "hal_gpio.h"

void hal_gpio_init(void)
{
    gpio_config_t io_conf = {0};

    /* LED */
    io_conf.pin_bit_mask  = (1ULL << HAL_GPIO_LED);
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(HAL_GPIO_LED, 0);

    /* BNO085 RESET – output, keep high */
    io_conf.pin_bit_mask  = (1ULL << HAL_GPIO_BNO_RST);
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level(HAL_GPIO_BNO_RST, 1);

    /* BNO085 INT – input, pull-up, polled */
    io_conf.pin_bit_mask  = (1ULL << HAL_GPIO_BNO_INT);
    io_conf.mode          = GPIO_MODE_INPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}
