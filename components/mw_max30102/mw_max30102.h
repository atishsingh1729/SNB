/*
 * mw_max30102.h
 * Middleware – MAX30102 heart-rate sensor.
 */

#ifndef MW_MAX30102_H_
#define MW_MAX30102_H_

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "sensor_types.h"

/* ── Registers ───────────────────────────────────────────────── */
#define MAX_REG_INTR_STATUS_1   0x00
#define MAX_REG_INTR_ENABLE_1   0x02
#define MAX_REG_INTR_ENABLE_2   0x03
#define MAX_REG_FIFO_WR_PTR     0x04
#define MAX_REG_OVF_COUNTER     0x05
#define MAX_REG_FIFO_RD_PTR     0x06
#define MAX_REG_FIFO_DATA       0x07
#define MAX_REG_FIFO_CONFIG     0x08
#define MAX_REG_MODE_CONFIG     0x09
#define MAX_REG_SPO2_CONFIG     0x0A
#define MAX_REG_LED1_PA         0x0C
#define MAX_REG_LED2_PA         0x0D

/* ── HR algorithm params ──────────────────────────────────────── */
#define MAX_RMS_WINDOW          50
#define MAX_HRV_WINDOW          30

/* ── API ─────────────────────────────────────────────────────── */
void    mw_max30102_init(i2c_master_dev_handle_t dev);
esp_err_t mw_max30102_write(uint8_t reg, uint8_t data);
esp_err_t mw_max30102_read_reg(uint8_t reg, uint8_t *data);
esp_err_t mw_max30102_read_fifo(uint8_t *buf, size_t len);

/**
 * @brief  Process one IR sample through the peak-detection algorithm.
 *         Updates heart_rate and hrv inside *out.
 */
void mw_max30102_process_sample(uint32_t ir, sensor_raw_t *out);

/**
 * @brief  Read all pending FIFO samples and update *out.
 *         Called by raw_data_reader in LIVE mode.
 */
void mw_max30102_get_raw(sensor_raw_t *out);

#endif /* MW_MAX30102_H_ */
