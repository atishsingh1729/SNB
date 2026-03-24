/*
 * file_reader.h  –  Record/replay sensor frames on W25Q64JV external flash
 *
 * FLASH LAYOUT
 * ────────────
 *   0x000000  WiFi config (4 KB sector, managed by mw_flash)
 *   0x001000  Session directory — 8 x 128-byte entries = 1 KB
 *   0x002000  Session data — sessions stored back-to-back in 4 KB sectors
 *
 * BINARY FRAME  (FR_FRAME_SIZE = 92 bytes)
 * ─────────────────────────────────────────
 *   float  accel x,y,z        12
 *   float  gyro  x,y,z        12
 *   float  mag   x,y,z        12
 *   float  lin_accel x,y,z    12
 *   float  roll,pitch,yaw     12
 *   u32    total_steps          4
 *   u32    raw_ir               4
 *   u32    heart_rate           4
 *   float  hrv                  4
 *   float  body_temp            4
 *   float  ambient_temp         4
 *   float  batt_voltage         4
 *   float  batt_percent         4   = 92 bytes total
 */

#ifndef FILE_READER_H_
#define FILE_READER_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sensor_types.h"

#define FR_ADDR_DIR           0x001000UL
#define FR_ADDR_DATA          0x002000UL
#define FR_MAX_SESSIONS       8
#define FR_SESSION_MAX_BYTES  (4UL * 1024 * 1024)
#define FR_FRAME_SIZE         92
#define FR_DIR_MAGIC          0xFACE0001UL

typedef struct {
    uint32_t magic;
    char     name[32];
    uint32_t start_addr;
    uint32_t frame_count;
    uint32_t sample_rate_hz;
    uint8_t  _pad[76];
} __attribute__((packed)) fr_dir_entry_t;   /* 128 bytes exactly */

/* No-op kept for API compatibility — external flash needs no mount */
esp_err_t file_reader_mount(void);

esp_err_t file_reader_open_write(const char *session_name);
esp_err_t file_reader_write_frame(const sensor_raw_t *frame);
esp_err_t file_reader_open_read(const char *session_name);
bool      file_reader_read_frame(sensor_raw_t *out);
esp_err_t file_reader_rewind(void);
void      file_reader_close(void);
void      file_reader_list_sessions(void);
esp_err_t file_reader_delete_session(const char *session_name);

#endif /* FILE_READER_H_ */
