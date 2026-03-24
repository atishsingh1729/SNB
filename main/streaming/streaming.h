/*
 * streaming.h
 *
 * ARCHITECTURE LAYER: Streaming Module
 * ────────────────────────────────────────────────────────────────
 * Owns all outbound data transport:
 *   - BLE GATT notifications
 *   - UDP broadcast (multiple ports for different data types)
 *   - Future: MQTT / HTTP POST
 *
 * Takes a data_snapshot_t + the JSON string and distributes them.
 * No sensor logic or state machine here.
 */

#ifndef STREAMING_H_
#define STREAMING_H_

#include <stdint.h>
#include "esp_err.h"
#include "data_pipeline.h"

/* ── UDP port assignments ─────────────────────────────────────── */
#define STREAM_UDP_PORT_IMU         5000
#define STREAM_UDP_PORT_HR          5001
#define STREAM_UDP_PORT_ADC         5002
#define STREAM_UDP_PORT_MOTION      5004
#define STREAM_UDP_PORT_VITALS      5005  /* HR/HRV/rawIR/Resp/Temp/bat */
#define STREAM_UDP_PORT_JSON        5006
#define STREAM_UDP_PORT_STATE       5007
#define STREAM_UDP_PORT_ORIENTATION 5008

/* ── API ─────────────────────────────────────────────────────── */

/** Initialise WiFi STA (reads creds from external flash) + BLE + UDP tasks. */
void streaming_init(void);

/** Send JSON snapshot over BLE to all subscribed clients. */
void streaming_ble_send(const char *json_str);

/** Update internal snapshot used by UDP tasks (call once per pipeline tick). */
void streaming_update_snapshot(const data_snapshot_t *snap,
                                const char            *json_str);

/**
 * @brief  Save new WiFi credentials + device name to external flash.
 *         The new config takes effect on next reboot.
 *         Called by the UDP tool config page handler.
 */
esp_err_t streaming_save_wifi_config(const char *ssid,
                                     const char *pass,
                                     const char *device_name);

/** Return current device name (loaded from flash or default). */
const char *streaming_get_device_name(void);

/** Return device MAC as "AA:BB:CC:DD:EE:FF" string (unique identifier). */
const char *streaming_get_mac_str(void);

#endif /* STREAMING_H_ */
