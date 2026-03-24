/*
 * file_reader.c  –  Record/replay sensor frames on W25Q64JV external flash
 *
 * All I/O goes through mw_flash_read() / mw_flash_write_page() /
 * mw_flash_sector_erase().  No SPIFFS, no internal flash.
 *
 * Write path: frames are buffered in a 256-byte page buffer and flushed to
 * flash when the buffer is full (one flash page write per 256 bytes = ~2.7
 * frames; leftover bytes are flushed on close).
 *
 * Read path: reads one page at a time into a small cache for efficiency.
 */

#include "file_reader.h"
#include "mw_flash.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "FILE_READER";

/* ── State ───────────────────────────────────────────────────── */
static bool     s_open       = false;
static bool     s_write_mode = false;

/* Write state */
static uint32_t s_write_addr  = 0;   /* next flash address to write  */
static uint32_t s_frame_count = 0;
static int      s_dir_slot    = -1;  /* which directory slot we own  */

/* 256-byte page write buffer */
static uint8_t  s_page_buf[256];
static uint16_t s_page_used  = 0;   /* bytes filled in page_buf      */
static uint32_t s_page_addr  = 0;   /* flash address of current page */

/* Read state */
static uint32_t s_read_base  = 0;   /* flash address of first frame  */
static uint32_t s_read_total = 0;   /* total frames in session       */
static uint32_t s_read_idx   = 0;   /* next frame index              */

/* ── Helpers ─────────────────────────────────────────────────── */
static void dir_read(int slot, fr_dir_entry_t *e)
{
    mw_flash_read(FR_ADDR_DIR + (uint32_t)slot * 128,
                  (uint8_t *)e, sizeof(fr_dir_entry_t));
}

static esp_err_t dir_write(int slot, const fr_dir_entry_t *e)
{
    /* Directory fits in one 4 KB sector (0x001000).
     * Must erase the whole sector then rewrite all 8 slots. */
    fr_dir_entry_t all[FR_MAX_SESSIONS];
    for (int i = 0; i < FR_MAX_SESSIONS; i++)
        dir_read(i, &all[i]);
    all[slot] = *e;

    esp_err_t ret = mw_flash_sector_erase(FR_ADDR_DIR);
    if (ret != ESP_OK) return ret;

    /* Write 8 * 128 = 1024 bytes in four 256-byte pages */
    uint8_t *src = (uint8_t *)all;
    for (int pg = 0; pg < 4; pg++) {
        ret = mw_flash_write_page(FR_ADDR_DIR + (uint32_t)pg * 256,
                                  src + pg * 256, 256);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static int find_slot_by_name(const char *name)
{
    fr_dir_entry_t e;
    for (int i = 0; i < FR_MAX_SESSIONS; i++) {
        dir_read(i, &e);
        if (e.magic == FR_DIR_MAGIC && strcmp(e.name, name) == 0)
            return i;
    }
    return -1;
}

static int find_free_slot(void)
{
    fr_dir_entry_t e;
    for (int i = 0; i < FR_MAX_SESSIONS; i++) {
        dir_read(i, &e);
        if (e.magic != FR_DIR_MAGIC)
            return i;
    }
    return -1;
}

/* Next free data address after all existing sessions */
static uint32_t next_free_data_addr(void)
{
    uint32_t end = FR_ADDR_DATA;
    fr_dir_entry_t e;
    for (int i = 0; i < FR_MAX_SESSIONS; i++) {
        dir_read(i, &e);
        if (e.magic != FR_DIR_MAGIC) continue;
        uint32_t session_end = e.start_addr
            + (uint32_t)e.frame_count * FR_FRAME_SIZE;
        /* Round up to next 4 KB sector boundary */
        session_end = (session_end + 0xFFF) & ~0xFFF;
        if (session_end > end) end = session_end;
    }
    return end;
}

/* Flush current page buffer to flash */
static esp_err_t flush_page(void)
{
    if (s_page_used == 0) return ESP_OK;

    /* Pad remainder with 0xFF (erased flash value) */
    memset(s_page_buf + s_page_used, 0xFF, 256 - s_page_used);

    esp_err_t ret = mw_flash_write_page(s_page_addr, s_page_buf, 256);
    if (ret == ESP_OK) {
        s_page_addr += 256;
        s_page_used  = 0;
        memset(s_page_buf, 0xFF, 256);
    }
    return ret;
}

/* Serialise one sensor_raw_t into FR_FRAME_SIZE bytes */
static void frame_to_bytes(const sensor_raw_t *f, uint8_t *buf)
{
    uint8_t *p = buf;
#define PUT_F32(v) do { float _v=(v); memcpy(p,&_v,4); p+=4; } while(0)
#define PUT_U32(v) do { uint32_t _v=(v); memcpy(p,&_v,4); p+=4; } while(0)
    PUT_F32(f->accel_x);   PUT_F32(f->accel_y);   PUT_F32(f->accel_z);
    PUT_F32(f->gyro_x);    PUT_F32(f->gyro_y);    PUT_F32(f->gyro_z);
    PUT_F32(f->mag_x);     PUT_F32(f->mag_y);     PUT_F32(f->mag_z);
    PUT_F32(f->lin_accel_x); PUT_F32(f->lin_accel_y); PUT_F32(f->lin_accel_z);
    PUT_F32(f->roll);      PUT_F32(f->pitch);     PUT_F32(f->yaw);
    PUT_U32(f->total_steps);
    PUT_U32(f->raw_ir);
    PUT_U32(f->heart_rate);
    PUT_F32(f->hrv);
    PUT_F32(f->body_temp);
    PUT_F32(f->ambient_temp);
    PUT_F32(f->batt_voltage);
    PUT_F32(f->batt_percent);
#undef PUT_F32
#undef PUT_U32
}

/* Deserialise FR_FRAME_SIZE bytes into sensor_raw_t */
static void bytes_to_frame(const uint8_t *buf, sensor_raw_t *f)
{
    const uint8_t *p = buf;
#define GET_F32(v) do { memcpy(&(v),p,4); p+=4; } while(0)
#define GET_U32(v) do { memcpy(&(v),p,4); p+=4; } while(0)
    GET_F32(f->accel_x);   GET_F32(f->accel_y);   GET_F32(f->accel_z);
    GET_F32(f->gyro_x);    GET_F32(f->gyro_y);    GET_F32(f->gyro_z);
    GET_F32(f->mag_x);     GET_F32(f->mag_y);     GET_F32(f->mag_z);
    GET_F32(f->lin_accel_x); GET_F32(f->lin_accel_y); GET_F32(f->lin_accel_z);
    GET_F32(f->roll);      GET_F32(f->pitch);     GET_F32(f->yaw);
    GET_U32(f->total_steps);
    GET_U32(f->raw_ir);
    GET_U32(f->heart_rate);
    GET_F32(f->hrv);
    GET_F32(f->body_temp);
    GET_F32(f->ambient_temp);
    GET_F32(f->batt_voltage);
    GET_F32(f->batt_percent);
#undef GET_F32
#undef GET_U32
}

/* ── Public API ──────────────────────────────────────────────── */

esp_err_t file_reader_mount(void)
{
    /* External flash needs no OS mount — already ready after mw_flash_init() */
    ESP_LOGI(TAG, "External flash session store ready (no mount needed)");
    return ESP_OK;
}

esp_err_t file_reader_open_write(const char *session_name)
{
    if (s_open) file_reader_close();

    /* Reject duplicate names */
    if (find_slot_by_name(session_name) >= 0) {
        ESP_LOGW(TAG, "Session '%s' already exists — delete it first",
                 session_name);
        return ESP_ERR_INVALID_ARG;
    }

    int slot = find_free_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "Directory full (%d sessions max)", FR_MAX_SESSIONS);
        return ESP_ERR_NO_MEM;
    }

    uint32_t data_addr = next_free_data_addr();

    /* Erase enough sectors for a reasonable session (erase lazily as needed) */
    /* First sector erase so first write succeeds */
    esp_err_t ret = mw_flash_sector_erase(data_addr);
    if (ret != ESP_OK) return ret;

    /* Write tentative directory entry (frame_count = 0 until close) */
    fr_dir_entry_t e = {0};
    e.magic          = FR_DIR_MAGIC;
    e.start_addr     = data_addr;
    e.frame_count    = 0;
    e.sample_rate_hz = 100;
    strncpy(e.name, session_name, sizeof(e.name) - 1);

    ret = dir_write(slot, &e);
    if (ret != ESP_OK) return ret;

    s_dir_slot    = slot;
    s_write_addr  = data_addr;
    s_frame_count = 0;
    s_page_addr   = data_addr;
    s_page_used   = 0;
    memset(s_page_buf, 0xFF, 256);
    s_open        = true;
    s_write_mode  = true;

    ESP_LOGI(TAG, "Recording '%s' @ 0x%06lX", session_name, data_addr);
    return ESP_OK;
}

esp_err_t file_reader_write_frame(const sensor_raw_t *frame)
{
    if (!s_open || !s_write_mode) return ESP_ERR_INVALID_STATE;

    uint8_t fbuf[FR_FRAME_SIZE];
    frame_to_bytes(frame, fbuf);

    uint16_t written = 0;
    while (written < FR_FRAME_SIZE) {
        uint16_t space = 256 - s_page_used;
        uint16_t chunk = (uint16_t)(FR_FRAME_SIZE - written);
        if (chunk > space) chunk = space;

        memcpy(s_page_buf + s_page_used, fbuf + written, chunk);
        s_page_used += chunk;
        written     += chunk;

        if (s_page_used == 256) {
            /* Before writing a new page, ensure the sector is erased.
             * A new sector starts every 4096 bytes. */
            uint32_t next_page_addr = s_page_addr + 256;
            if ((next_page_addr & 0xFFF) == 0) {
                mw_flash_sector_erase(next_page_addr);
            }
            esp_err_t ret = flush_page();
            if (ret != ESP_OK) return ret;
        }
    }

    s_frame_count++;
    return ESP_OK;
}

esp_err_t file_reader_open_read(const char *session_name)
{
    if (s_open) file_reader_close();

    int slot = find_slot_by_name(session_name);
    if (slot < 0) {
        ESP_LOGE(TAG, "Session '%s' not found", session_name);
        return ESP_ERR_NOT_FOUND;
    }

    fr_dir_entry_t e;
    dir_read(slot, &e);

    s_read_base  = e.start_addr;
    s_read_total = e.frame_count;
    s_read_idx   = 0;
    s_open       = true;
    s_write_mode = false;

    ESP_LOGI(TAG, "Replaying '%s' — %lu frames", session_name, e.frame_count);
    return ESP_OK;
}

bool file_reader_read_frame(sensor_raw_t *out)
{
    if (!s_open || s_write_mode || !out) return false;
    if (s_read_idx >= s_read_total)       return false;

    uint32_t addr = s_read_base + s_read_idx * FR_FRAME_SIZE;
    uint8_t  buf[FR_FRAME_SIZE];

    esp_err_t ret = mw_flash_read(addr, buf, FR_FRAME_SIZE);
    if (ret != ESP_OK) return false;

    bytes_to_frame(buf, out);
    s_read_idx++;
    return true;
}

esp_err_t file_reader_rewind(void)
{
    if (!s_open || s_write_mode) return ESP_ERR_INVALID_STATE;
    s_read_idx = 0;
    return ESP_OK;
}

void file_reader_close(void)
{
    if (!s_open) return;

    if (s_write_mode) {
        /* Flush remaining partial page */
        flush_page();

        /* Update frame count in directory */
        if (s_dir_slot >= 0) {
            fr_dir_entry_t e;
            dir_read(s_dir_slot, &e);
            e.frame_count = s_frame_count;
            dir_write(s_dir_slot, &e);
        }
        ESP_LOGI(TAG, "Closed write — %lu frames saved", s_frame_count);
    }

    s_open       = false;
    s_write_mode = false;
    s_dir_slot   = -1;
    s_frame_count = 0;
}

void file_reader_list_sessions(void)
{
    ESP_LOGI(TAG, "Sessions on external flash:");
    fr_dir_entry_t e;
    for (int i = 0; i < FR_MAX_SESSIONS; i++) {
        dir_read(i, &e);
        if (e.magic != FR_DIR_MAGIC) continue;
        float secs = (float)e.frame_count / (float)(e.sample_rate_hz ? e.sample_rate_hz : 100);
        ESP_LOGI(TAG, "  [%d] '%s'  frames=%lu  dur=%.1fs  addr=0x%06lX",
                 i, e.name, e.frame_count, secs, e.start_addr);
    }
}

esp_err_t file_reader_delete_session(const char *session_name)
{
    int slot = find_slot_by_name(session_name);
    if (slot < 0) {
        ESP_LOGE(TAG, "Session '%s' not found", session_name);
        return ESP_ERR_NOT_FOUND;
    }

    fr_dir_entry_t e;
    dir_read(slot, &e);
    memset(&e, 0xFF, sizeof(e));   /* 0xFF = erased = free slot */
    dir_write(slot, &e);

    ESP_LOGI(TAG, "Session '%s' deleted (slot %d freed)", session_name, slot);
    return ESP_OK;
}
