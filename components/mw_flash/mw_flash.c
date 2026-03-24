/*
 * mw_flash.c  –  W25Q64JV external SPI flash middleware
 */

#include "mw_flash.h"
#include "hal_spi.h"
#include "esp_log.h"
#include "esp_efuse.h"
#include <string.h>

static const char *TAG = "MW_FLASH";

/* ── Init ────────────────────────────────────────────────────── */
void mw_flash_init(void)
{
    ESP_LOGI(TAG, "W25Q64JV flash middleware ready");
}

/* ── JEDEC ID ────────────────────────────────────────────────── */
void mw_flash_read_jedec_id(void)
{
    uint8_t tx[4] = {FLASH_CMD_JEDEC_ID, 0, 0, 0};
    uint8_t rx[4] = {0};
    spi_transaction_t t = { .length = 32, .tx_buffer = tx, .rx_buffer = rx };
    if (spi_device_transmit(hal_spi_flash, &t) == ESP_OK)
        ESP_LOGI(TAG, "JEDEC ID: %02X %02X %02X", rx[1], rx[2], rx[3]);
    else
        ESP_LOGE(TAG, "JEDEC read failed");
}

/* ── Wait for WIP bit to clear ───────────────────────────────── */
static esp_err_t wait_not_busy(void)
{
    uint8_t tx[2] = {FLASH_CMD_READ_SR1, 0};
    uint8_t rx[2] = {0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    for (int i = 0; i < 10000; i++) {
        spi_device_transmit(hal_spi_flash, &t);
        if (!(rx[1] & 0x01)) return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

/* ── Write Enable ────────────────────────────────────────────── */
static esp_err_t write_enable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_EN;
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    return spi_device_transmit(hal_spi_flash, &t);
}

/* ── Read ────────────────────────────────────────────────────── */
esp_err_t mw_flash_read(uint32_t addr, uint8_t *buf, size_t len)
{
    uint8_t cmd[4] = {
        FLASH_CMD_READ,
        (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr
    };
    uint8_t full[4 + len];
    memset(full, 0, sizeof(full));
    memcpy(full, cmd, 4);

    spi_transaction_t t = {
        .length   = 8 * (4 + len),
        .tx_buffer = full,
        .rx_buffer = full,
    };
    esp_err_t ret = spi_device_transmit(hal_spi_flash, &t);
    if (ret == ESP_OK)
        memcpy(buf, full + 4, len);
    return ret;
}

/* ── Sector Erase (4 KB) ─────────────────────────────────────── */
esp_err_t mw_flash_sector_erase(uint32_t addr)
{
    esp_err_t ret = write_enable();
    if (ret != ESP_OK) return ret;

    uint8_t cmd[4] = {
        FLASH_CMD_SECTOR_ERASE,
        (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr
    };
    spi_transaction_t t = { .length = 32, .tx_buffer = cmd };
    ret = spi_device_transmit(hal_spi_flash, &t);
    if (ret != ESP_OK) return ret;
    return wait_not_busy();
}

/* ── Page Program (up to 256 bytes) ──────────────────────────── */
esp_err_t mw_flash_write_page(uint32_t addr, const uint8_t *data, size_t len)
{
    if (len > 256) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = write_enable();
    if (ret != ESP_OK) return ret;

    uint8_t buf[260];
    buf[0] = FLASH_CMD_PAGE_PROG;
    buf[1] = (uint8_t)(addr >> 16);
    buf[2] = (uint8_t)(addr >> 8);
    buf[3] = (uint8_t)addr;
    memcpy(buf + 4, data, len);

    spi_transaction_t t = { .length = 8 * (4 + len), .tx_buffer = buf };
    ret = spi_device_transmit(hal_spi_flash, &t);
    if (ret != ESP_OK) return ret;
    return wait_not_busy();
}

/* ── Config: load from flash ─────────────────────────────────── */
bool mw_flash_config_load(flash_wifi_config_t *cfg)
{
    esp_err_t ret = mw_flash_read(FLASH_ADDR_CONFIG,
                                  (uint8_t *)cfg,
                                  sizeof(flash_wifi_config_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config read failed: %s", esp_err_to_name(ret));
        return false;
    }
    if (cfg->magic != FLASH_CONFIG_MAGIC) {
        ESP_LOGW(TAG, "No valid config in flash (magic=0x%08lX)", cfg->magic);
        return false;
    }
    /* Populate MAC at runtime using IDF 5.x API */
    esp_efuse_mac_get_default(cfg->mac);
    ESP_LOGI(TAG, "Config loaded: SSID='%s' Device='%s'",
             cfg->ssid, cfg->device_name);
    return true;
}

/* ── Config: save to flash ───────────────────────────────────── */
esp_err_t mw_flash_config_save(const flash_wifi_config_t *cfg)
{
    /* Erase the config sector first */
    esp_err_t ret = mw_flash_sector_erase(FLASH_ADDR_CONFIG);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sector erase failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mw_flash_write_page(FLASH_ADDR_CONFIG,
                               (const uint8_t *)cfg,
                               sizeof(flash_wifi_config_t));
    if (ret == ESP_OK)
        ESP_LOGI(TAG, "Config saved: SSID='%s' Device='%s'",
                 cfg->ssid, cfg->device_name);
    else
        ESP_LOGE(TAG, "Config write failed: %s", esp_err_to_name(ret));
    return ret;
}
