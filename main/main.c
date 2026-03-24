/*
 * main.c
 *
 * SmartNeckBand – ESP32-S3
 * Architecture:
 *
 *  ┌──────────────────────────────────────────────────────────┐
 *  │           Application / Feature Processing                │
 *  │         (Dehydration, Step Count, State Machine)          │
 *  ├─────────────────────────┬────────────────────────────────┤
 *  │       Data Module       │       Streaming Module         │
 *  ├─────────────────────────┴────────────────────────────────┤
 *  │                  Sensor Processing                        │
 *  ├────────────────────────────┬─────────────────────────────┤
 *  │   Middleware / Components  │      Raw Data Reader        │
 *  │  BNO085 MAX30102 NTC       │  (LIVE / FILE / UDP /       │
 *  │  Audio  Flash  BLE WiFi   │   STATIC – swappable)       │
 *  ├────────────────────────────┴─────────────────────────────┤
 *  │              HAL  (ADC · I2C · SPI · GPIO)               │
 *  └──────────────────────────────────────────────────────────┘
 *
 * ══════════════════════════════════════════════════════════════
 *  HOW TO SWITCH DATA SOURCE  (3 defines below)
 * ══════════════════════════════════════════════════════════════
 *
 *  Normal – live sensors:
 *      DATA_SOURCE   = DATA_SRC_LIVE
 *      OVERRIDE_MASK = 0
 *
 *  Replay a recorded session from external flash:
 *      DATA_SOURCE   = DATA_SRC_FILE
 *      OVERRIDE_MASK = RAW_SRC_ALL
 *      INJECT_FILE   = "dog01_run1"        <-- session name (no path/ext)
 *
 *  Inject only IMU from file, keep HR + NTC live:
 *      DATA_SOURCE   = DATA_SRC_FILE
 *      OVERRIDE_MASK = RAW_SRC_BNO085
 *
 *  Feed data from PC over UDP (port 6000):
 *      DATA_SOURCE   = DATA_SRC_UDP
 *      OVERRIDE_MASK = RAW_SRC_ALL
 *
 *  Record a new session to external flash:
 *      AUTO_RECORD   = 1
 *      RECORD_NAME   = "dog01_run1"
 * ══════════════════════════════════════════════════════════════
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

/* ── HAL ──────────────────────────────────────────────────────── */
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "hal_spi.h"
#include "hal_adc.h"

/* ── Middleware ───────────────────────────────────────────────── */
#include "mw_bno085.h"
#include "mw_max30102.h"
#include "mw_ntc.h"
#include "mw_flash.h"

/* ── Data source ──────────────────────────────────────────────── */
#include "raw_data_reader.h"
#include "file_reader.h"

/* ── Processing layers ────────────────────────────────────────── */
#include "sensor_processing.h"
#include "data_pipeline.h"
#include "streaming.h"

/* ── App layer ────────────────────────────────────────────────── */
#include "feature_processing.h"

/* ── Shared types ─────────────────────────────────────────────── */
#include "sensor_types.h"

static const char *TAG = "MAIN";

/* ════════════════════════════════════════════════════════════════
 *  DATA SOURCE CONFIGURATION
 *  Change these 3 defines to switch between live / file / UDP.
 * ════════════════════════════════════════════════════════════════ */
#define DATA_SOURCE     DATA_SRC_LIVE   /* DATA_SRC_LIVE | FILE | UDP | STATIC */
#define OVERRIDE_MASK   0               /* 0 = all live; RAW_SRC_ALL = all injected */
#define INJECT_FILE     "dog01_run1"    /* session name stored in external flash */

/* Set 1 to start a recording session automatically on boot */
#define AUTO_RECORD     0
#define RECORD_NAME     "dog01_run1"

/* ════════════════════════════════════════════════════════════════
 *  Shared pipeline state (mutex-protected)
 * ════════════════════════════════════════════════════════════════ */
static SemaphoreHandle_t  s_mutex   = NULL;
static sensor_raw_t       s_raw     = {0};
static sensor_processed_t s_proc    = {0};
static data_snapshot_t    s_snap    = {0};
static char               s_json[1200] = {0};

/* ════════════════════════════════════════════════════════════════
 *  TASK: LED heartbeat
 * ════════════════════════════════════════════════════════════════ */
static void task_led(void *arg)
{
    while (1) {
        gpio_set_level(HAL_GPIO_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(HAL_GPIO_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ════════════════════════════════════════════════════════════════
 *  TASK: Temperature + Battery  (slow – 3 s period)
 *  Only runs when NTC is not overridden by inject source.
 * ════════════════════════════════════════════════════════════════ */
static void task_temperature(void *arg)
{
    while (1) {
        if (!(OVERRIDE_MASK & RAW_SRC_NTC)) {
            sensor_raw_t tmp = {0};
            mw_ntc_get_raw(&tmp);

            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                s_raw.body_temp    = tmp.body_temp;
                s_raw.ambient_temp = tmp.ambient_temp;
                s_raw.batt_voltage = tmp.batt_voltage;
                s_raw.batt_percent = tmp.batt_percent;
                xSemaphoreGive(s_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ════════════════════════════════════════════════════════════════
 *  TASK: Main sensor + processing loop  (100 Hz / 10 ms)
 *  Reads raw data → sensor processing → state machine.
 * ════════════════════════════════════════════════════════════════ */
static void task_sensor(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    /* Persistent frames — fields accumulate, never zero between ticks */
    static sensor_raw_t       frame = {0};
    static sensor_processed_t proc  = {0};

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        /* 1. Fetch raw frame (live or injected).
              mw_bno085_get_raw() now returns an ACCUMULATED frame
              so fields not present in this packet keep last value. */
        if (!raw_data_reader_get(&frame)) continue;

        /* 2. Merge slow-update NTC fields (no timeout risk — use portMAX_DELAY) */
        if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
            /* Only overwrite temp/batt if NTC task has written at least once */
            if (s_raw.body_temp != 0.0f)
                frame.body_temp = s_raw.body_temp;
            if (s_raw.ambient_temp != 0.0f)
                frame.ambient_temp = s_raw.ambient_temp;
            if (s_raw.batt_voltage != 0.0f) {
                frame.batt_voltage = s_raw.batt_voltage;
                frame.batt_percent = s_raw.batt_percent;
            }
            xSemaphoreGive(s_mutex);
        }

        /* 3. Sensor processing: magnitudes, ROM, FFT respiration */
        sensor_processing_run(&frame, &proc);
        sensor_processing_fft_tick(&frame, &proc);

        /* 4. Feed state machine */
        feature_processing_feed(&sm, &frame, &proc);
        process_state_transitions(&sm);

        /* 5. Publish for streaming task */
        if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&s_raw,  &frame, sizeof(sensor_raw_t));
            memcpy(&s_proc, &proc,  sizeof(sensor_processed_t));
            xSemaphoreGive(s_mutex);
        }
    }
}

/* ════════════════════════════════════════════════════════════════
 *  TASK: Streaming  (1 s period)
 *  Builds snapshot → JSON → BLE notify + UDP broadcast.
 * ════════════════════════════════════════════════════════════════ */
static void task_streaming(void *arg)
{
    while (1) {
        sensor_raw_t       raw  = {0};
        sensor_processed_t proc = {0};

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&raw,  &s_raw,  sizeof(sensor_raw_t));
            memcpy(&proc, &s_proc, sizeof(sensor_processed_t));
            xSemaphoreGive(s_mutex);
        }

        /* Build unified snapshot */
        data_pipeline_build_snapshot(&raw, &proc,
                                     (uint8_t)sm.current_state,
                                     &s_snap);

        /* Serialise to JSON */
        data_pipeline_to_json(&s_snap, s_json, sizeof(s_json));

        /* Distribute over BLE + UDP */
        streaming_update_snapshot(&s_snap, s_json);
        streaming_ble_send(s_json);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ════════════════════════════════════════════════════════════════
 *  app_main
 * ════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "=============================");
    ESP_LOGI(TAG, "   SmartNeckBand booting...  ");
    ESP_LOGI(TAG, "=============================");

    /* ── Shared mutex ──────────────────────────────────────── */
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    /* ══════════════════════════════════════════════════════
     *  HAL INIT
     * ══════════════════════════════════════════════════════ */
    hal_gpio_init();
    hal_i2c_init();          /* also scans bus + creates device handles */
    hal_spi_init();
    hal_adc_init();
    hal_adc_init_calibration();
    ESP_LOGI(TAG, "[HAL] All peripherals initialised");

    /* ══════════════════════════════════════════════════════
     *  EXTERNAL FLASH INIT (must be before file_reader and streaming)
     * ══════════════════════════════════════════════════════ */
    mw_flash_init();
    mw_flash_read_jedec_id();

    /* List sessions stored on external flash */
    file_reader_mount();              /* no-op — kept for compatibility */
    file_reader_list_sessions();

    /* ══════════════════════════════════════════════════════
     *  MIDDLEWARE INIT
     * ══════════════════════════════════════════════════════ */
    vTaskDelay(pdMS_TO_TICKS(200));   /* let sensors power up */

    /* Flash already initialised above */

    /* IMU – BNO085 */
    mw_bno085_init(hal_i2c_bno085);
    mw_bno085_enable_feature(BNO_REPORT_ACCEL,      10000);
    mw_bno085_enable_feature(BNO_REPORT_GYRO,       10000);
    mw_bno085_enable_feature(BNO_REPORT_LIN_ACCEL,  10000);
    mw_bno085_enable_feature(BNO_REPORT_MAG,        10000);
    mw_bno085_enable_feature(BNO_REPORT_STEPCOUNT,  10000);
    mw_bno085_enable_feature(BNO_REPORT_GAME_ROT,   10000);
    mw_bno085_enable_feature(BNO_REPORT_GYRO_ROT,   10000);

    /* Heart-rate – MAX30102 */
    mw_max30102_init(hal_i2c_max30102);

    /* Temperature + battery – NTC */
    mw_ntc_init();

    ESP_LOGI(TAG, "[MIDDLEWARE] All sensors initialised");

    /* ══════════════════════════════════════════════════════
     *  RAW DATA READER  (switchable data source)
     * ══════════════════════════════════════════════════════ */
    raw_data_reader_cfg_t rdr_cfg = {
        .source           = DATA_SOURCE,
        .inject_file_path = INJECT_FILE,
        .udp_listen_port  = 6000,
        .override_mask    = OVERRIDE_MASK,
    };
    raw_data_reader_init(&rdr_cfg);

    ESP_LOGI(TAG, "[RAW_DATA_READER] source=%s  override_mask=0x%02X",
             raw_data_reader_source_name(DATA_SOURCE), OVERRIDE_MASK);

    /* ══════════════════════════════════════════════════════
     *  SENSOR PROCESSING
     * ══════════════════════════════════════════════════════ */
    sensor_processing_init();

    /* ══════════════════════════════════════════════════════
     *  DATA PIPELINE
     * ══════════════════════════════════════════════════════ */
    data_pipeline_init("NECKBAND_01");

#if AUTO_RECORD
    data_pipeline_start_recording(RECORD_NAME);
    ESP_LOGI(TAG, "[PIPELINE] Auto-recording → %s", RECORD_NAME);
#endif

    /* ══════════════════════════════════════════════════════
     *  STATE MACHINE
     * ══════════════════════════════════════════════════════ */
    initialize_state_machine(&sm);

    /* Set system time (replace with NTP / cloud sync) */
    struct timeval tv = { .tv_sec = 1772185467, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    /* ══════════════════════════════════════════════════════
     *  STREAMING  (BLE + WiFi UDP)
     * ══════════════════════════════════════════════════════ */
    streaming_init();

    /* ══════════════════════════════════════════════════════
     *  TASKS
     * ══════════════════════════════════════════════════════
     *
     *  task_led         – 1 KB  pri 3   – LED blink
     *  task_temperature – 4 KB  pri 5   – NTC + battery (3 s)
     *  task_sensor      – 8 KB  pri 10  – 100 Hz main loop
     *  task_streaming   – 4 KB  pri 5   – 1 s JSON / BLE / UDP
     */
    xTaskCreate(task_led,         "led",    1024, NULL, 3,  NULL);
    xTaskCreate(task_temperature, "ntc",    4096, NULL, 5,  NULL);
    xTaskCreate(task_sensor,      "sensor", 8192, NULL, 10, NULL);
    xTaskCreate(task_streaming,   "stream", 4096, NULL, 5,  NULL);

    ESP_LOGI(TAG, "[MAIN] All tasks started – running.");

    /* Main task idles */
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}
