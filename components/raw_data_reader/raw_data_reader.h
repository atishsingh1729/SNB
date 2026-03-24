/*
 * raw_data_reader.h
 *
 * PURPOSE
 * ───────
 * This is the SWITCHABLE DATA SOURCE layer.
 *
 * In LIVE mode   → data comes from real BNO085 / MAX30102 / NTC sensors.
 * In INJECT mode → data is read from a CSV/binary file on flash or fed
 *                  over UDP from a PC, so you can replay real dog-test
 *                  recordings without any physical sensor attached.
 *
 * USAGE
 * ─────
 *  1. Call raw_data_reader_set_source() once at startup.
 *  2. Every sensor-processing tick calls raw_data_reader_get() to
 *     fill a sensor_raw_t – regardless of whether sensors are live.
 *
 * To disconnect a single sensor (e.g., MAX30102 broken on bench):
 *   raw_data_reader_set_sensor_override(RAW_SRC_MAX30102, true);
 *   raw_data_reader_load_inject_file("/spiffs/dog01_hr.csv");
 *
 * The rest of the pipeline sees no difference.
 */

#ifndef RAW_DATA_READER_H_
#define RAW_DATA_READER_H_

#include <stdbool.h>
#include <stdint.h>
#include "sensor_types.h"

/* ── Data source modes ───────────────────────────────────────── */
typedef enum {
    DATA_SRC_LIVE     = 0,  /* Read from real sensors via middleware   */
    DATA_SRC_FILE     = 1,  /* Replay from CSV file stored on SPIFFS   */
    DATA_SRC_UDP      = 2,  /* Receive injected frames over UDP (PC)   */
    DATA_SRC_STATIC   = 3,  /* Fixed test values (unit-test / demo)    */
} data_source_t;

/* ── Per-sensor override flags ───────────────────────────────── */
typedef enum {
    RAW_SRC_BNO085   = (1 << 0),
    RAW_SRC_MAX30102 = (1 << 1),
    RAW_SRC_NTC      = (1 << 2),
    RAW_SRC_ALL      = 0xFF,
} sensor_override_mask_t;

/* ── Configuration ───────────────────────────────────────────── */
typedef struct {
    data_source_t  source;

    /* FILE mode */
    const char    *inject_file_path;  /* e.g. "/spiffs/dog01.csv" */

    /* UDP mode */
    uint16_t       udp_listen_port;   /* default 6000             */

    /* STATIC mode – set these before calling init */
    sensor_raw_t   static_frame;

    /* Bitmask of sensors to override (rest stay LIVE) */
    uint8_t        override_mask;     /* sensor_override_mask_t   */
} raw_data_reader_cfg_t;

/* ── API ─────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the data reader with the given config.
 *         Call once after HAL + middleware are ready.
 */
void raw_data_reader_init(const raw_data_reader_cfg_t *cfg);

/**
 * @brief  Override source for specific sensors at runtime.
 *         Allows mixing: e.g. BNO085 from file, MAX30102 live.
 * @param  mask  Bitmask of sensors to redirect to inject source.
 * @param  src   Where to get the injected data from.
 */
void raw_data_reader_set_override(uint8_t mask, data_source_t src);

/**
 * @brief  Fill *out with the latest sensor frame.
 *         Transparently mixes live and injected sources.
 * @return true if frame is valid, false if no data available yet.
 */
bool raw_data_reader_get(sensor_raw_t *out);

/**
 * @brief  Push one raw frame from external code (e.g. UDP task).
 *         Only used when source == DATA_SRC_UDP.
 */
void raw_data_reader_push_udp_frame(const sensor_raw_t *frame);

/**
 * @brief  Utility: return human-readable source name.
 */
const char *raw_data_reader_source_name(data_source_t src);

#endif /* RAW_DATA_READER_H_ */
