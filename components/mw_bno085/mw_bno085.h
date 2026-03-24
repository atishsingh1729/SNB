/*
 * mw_bno085.h
 * Middleware – BNO085 IMU driver (SHTP over I2C).
 *
 * Exposes one function used by raw_data_reader in LIVE mode:
 *   mw_bno085_get_raw()
 *
 * All other functions are init / feature-enable helpers.
 */

#ifndef MW_BNO085_H_
#define MW_BNO085_H_

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "sensor_types.h"

/* ── SHTP / Report IDs ───────────────────────────────────────── */
#define BNO_MAX_PAYLOAD         512
#define BNO_HEADER_LEN          4
#define BNO_REPORT_ACCEL        0x01
#define BNO_REPORT_GYRO         0x02
#define BNO_REPORT_MAG          0x03
#define BNO_REPORT_LIN_ACCEL    0x04
#define BNO_REPORT_GAME_ROT     0x08
#define BNO_REPORT_STEPCOUNT    0x11
#define BNO_REPORT_HEART_RATE   0x23
#define BNO_REPORT_GYRO_ROT     0x2A

/* ── API ─────────────────────────────────────────────────────── */

/** Init – call after I2C bus is ready. */
void mw_bno085_init(i2c_master_dev_handle_t dev);

/** Enable a sensor report at the given interval (µs). */
void mw_bno085_enable_feature(uint8_t report_id, uint32_t interval_us);

/**
 * @brief  Read one SHTP packet and decode all embedded reports
 *         into *out.  Called in LIVE mode by raw_data_reader.
 */
void mw_bno085_get_raw(sensor_raw_t *out);

#endif /* MW_BNO085_H_ */
