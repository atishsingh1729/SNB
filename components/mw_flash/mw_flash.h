/*
 * mw_flash.h  –  W25Q64JV external SPI flash middleware
 *
 * Layout (all page-aligned, 256 bytes each):
 *   0x000000  – Config page  (wifi_config_t + magic)
 *   0x001000  – Data log page 0  (raw session data)
 *   ...
 */

#ifndef MW_FLASH_H_
#define MW_FLASH_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ── W25Q64JV commands ───────────────────────────────────────── */
#define FLASH_CMD_JEDEC_ID    0x9F
#define FLASH_CMD_READ        0x03
#define FLASH_CMD_PAGE_PROG   0x02
#define FLASH_CMD_WRITE_EN    0x06
#define FLASH_CMD_CHIP_ERASE  0x60
#define FLASH_CMD_SECTOR_ERASE 0x20   /* 4 KB sector erase */
#define FLASH_CMD_READ_SR1    0x05

/* ── Flash address map ───────────────────────────────────────── */
#define FLASH_ADDR_CONFIG     0x000000  /* 4KB sector — WiFi config  */
#define FLASH_ADDR_DATA_LOG   0x001000  /* Remaining — session data  */

/* ── Config magic ────────────────────────────────────────────── */
#define FLASH_CONFIG_MAGIC    0xA55A3CC3UL

/* ── WiFi config stored in flash ─────────────────────────────── */
typedef struct {
    uint32_t magic;            /* FLASH_CONFIG_MAGIC when valid     */
    char     ssid[32];
    char     password[64];
    char     device_name[24];  /* e.g. "NECKBAND_01"               */
    uint8_t  mac[6];           /* populated at runtime, not stored  */
    uint8_t  _pad[2];
} flash_wifi_config_t;         /* must fit in 256 bytes             */

/* ── API ─────────────────────────────────────────────────────── */
void      mw_flash_init(void);
void      mw_flash_read_jedec_id(void);
esp_err_t mw_flash_read(uint32_t addr, uint8_t *buf, size_t len);
esp_err_t mw_flash_write_page(uint32_t addr, const uint8_t *data, size_t len);
esp_err_t mw_flash_sector_erase(uint32_t addr);  /* erases 4 KB sector */

/* ── Config helpers ──────────────────────────────────────────── */
bool      mw_flash_config_load(flash_wifi_config_t *cfg);
esp_err_t mw_flash_config_save(const flash_wifi_config_t *cfg);

#endif /* MW_FLASH_H_ */
