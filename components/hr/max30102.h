/*
 * max30102.h
 *
 *  Created on: Nov 28, 2025
 *      Author: Dheeraj.Jain
 */

#ifndef MAIN_MAX30102_H_
#define MAIN_MAX30102_H_

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdint.h>

/* ============== MAX30102 REGISTERS ============== */
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
 
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03

#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG		0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D

/* ============== HR/HRV PARAMETERS =================== */
#define MIN_PEAK_INTERVAL_MS   300   // MS
#define HR_AVG_COUNT			8

#define RMS_WINDOW_SAMPLES 50  // 500ms window
#define HRV_WINDOW 30   // Number of beats for HRV calculation


/* ================= RESPIRATION CONFIG ================= */
#define RESP_MIN_INTERVAL_MS   1800     // max
#define RESP_MAX_INTERVAL_MS   8000    // min 7.5 BPM
#define RESP_SMOOTH_ALPHA      0.2f






void max30102_init(i2c_master_dev_handle_t dev);
esp_err_t max30102_read_fifo(uint8_t *data, size_t len);
esp_err_t max30102_write(uint8_t reg, uint8_t data);
esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data);
void hr_process_sample(uint32_t ir);

#endif /* MAIN_MAX30102_H_ */
