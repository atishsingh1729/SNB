#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>

#define SERVICE_UUID        0xFF00
#define CHARACTERISTIC_UUID 0xFF01
#define CHAR_VALUE_LEN      1024
#define MAX_CONNECTIONS     4

void ble_init(void);

void send_message_all(uint8_t *data, uint16_t length);
void send_message_to_conn(uint16_t conn_handle, uint8_t *data, uint16_t length);

/* Application callback */
void on_message_received(uint8_t *data, uint16_t len, uint16_t conn_handle);

#endif
