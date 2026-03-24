/*
 * sensor_processing.h
 *
 * ARCHITECTURE LAYER: Sensor Processing
 * ────────────────────────────────────────────────────────────────
 * Sits between the Raw Data Reader and the Data Pipeline/App layers.
 *
 * Responsibilities:
 *  - Compute derived quantities (magnitudes, variance, ROM, resp rate)
 *  - Run FFT-based respiration estimation
 *  - Write results into sensor_processed_t
 *  - Does NOT contain feature-level logic (no state machine here)
 *
 * Input:  sensor_raw_t    (from raw_data_reader_get)
 * Output: sensor_processed_t  (consumed by data_pipeline / app layer)
 */

#ifndef SENSOR_PROCESSING_H_
#define SENSOR_PROCESSING_H_

#include "sensor_types.h"

/* ── API ─────────────────────────────────────────────────────── */

/**
 * @brief  One-time init. Call after raw_data_reader_init().
 */
void sensor_processing_init(void);

/**
 * @brief  Process one raw frame into a processed frame.
 *         Call every sensor tick (≈10 ms / 100 Hz).
 *
 * @param  raw  Filled by raw_data_reader_get().
 * @param  out  Derived quantities written here.
 */
void sensor_processing_run(const sensor_raw_t *raw, sensor_processed_t *out);

/**
 * @brief  Run FFT respiration analysis on a rolling buffer of accel
 *         magnitudes.  Call at the same rate as sensor_processing_run().
 *         Updates out->respiration_rate when the buffer fills.
 */
void sensor_processing_fft_tick(const sensor_raw_t *raw, sensor_processed_t *out);

#endif /* SENSOR_PROCESSING_H_ */
