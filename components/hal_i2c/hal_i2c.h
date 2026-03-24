/*
 * hal_i2c.h
 * HAL Layer – I2C master bus abstraction.
 * All I2C device handles are created here and exported for middleware use.
 */

#ifndef HAL_I2C_H_
#define HAL_I2C_H_

#include "driver/i2c_master.h"
#include "driver/gpio.h"

/* ── Pin & speed config ──────────────────────────────────────── */
#define HAL_I2C_SDA_GPIO    GPIO_NUM_8
#define HAL_I2C_SCL_GPIO    GPIO_NUM_9
#define HAL_I2C_SPEED_HZ    100000

/* ── Device I2C addresses ────────────────────────────────────── */
#define HAL_I2C_ADDR_BNO085   0x4B
#define HAL_I2C_ADDR_MAX30102 0x57

/* ── Public handles ──────────────────────────────────────────── */
extern i2c_master_bus_handle_t hal_i2c_bus;
extern i2c_master_dev_handle_t hal_i2c_bno085;
extern i2c_master_dev_handle_t hal_i2c_max30102;

/* ── API ─────────────────────────────────────────────────────── */
void hal_i2c_init(void);

/** Scan and log all found I2C addresses (debug utility). */
void hal_i2c_scan(void);

#endif /* HAL_I2C_H_ */
