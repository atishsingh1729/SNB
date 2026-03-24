/*
 * streaming.c
 * Streaming Module – BLE + UDP broadcast.
 * Refactored from components/ble/ble.c  +  components/WiFi/wifi.c
 */

#include "streaming.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_efuse.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* NimBLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <string.h>
#include <stdio.h>

#include "mw_flash.h"

/* ── Runtime WiFi/device config (loaded from flash in streaming_init) ── */
/* Declared at file scope so ble_on_sync() and all UDP tasks can see them */
static char    s_wifi_ssid[32]   = "ksplaim";      /* fallback default */
static char    s_wifi_pass[64]   = "12345678";
static char    s_device_name[24] = "NECKBAND_01";
static uint8_t s_mac[6]          = {0};

static const char *TAG = "STREAMING";

/* ════════════════════════════════════════════════════════════════
 *  SHARED SNAPSHOT (written by pipeline, read by UDP tasks)
 * ════════════════════════════════════════════════════════════════ */
#define JSON_BUF_SIZE 1200

static data_snapshot_t s_snap;
static char            s_json_buf[JSON_BUF_SIZE];
static SemaphoreHandle_t s_snap_mutex = NULL;

void streaming_update_snapshot(const data_snapshot_t *snap, const char *json_str)
{
    if (xSemaphoreTake(s_snap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&s_snap, snap, sizeof(data_snapshot_t));
        if (json_str) {
            strncpy(s_json_buf, json_str, JSON_BUF_SIZE - 1);
            s_json_buf[JSON_BUF_SIZE - 1] = '\0';
        }
        xSemaphoreGive(s_snap_mutex);
    }
}

/* ════════════════════════════════════════════════════════════════
 *  BLE
 * ════════════════════════════════════════════════════════════════ */
#define BLE_MAX_CONN  4
#define BLE_CHAR_LEN  1024

typedef struct { uint16_t handle; bool in_use; bool notify_en; } ble_conn_t;
static ble_conn_t   s_conns[BLE_MAX_CONN];
static uint8_t      s_active_conns = 0;
static uint16_t     s_notify_handle = 0;
static bool         s_advertising   = false;
static uint8_t      s_own_addr_type;

static void ble_start_advertising(void);
static int  gap_event_cb(struct ble_gap_event *event, void *arg);

static int gatt_access_cb(uint16_t ch, uint16_t ah,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        static uint8_t rx[BLE_CHAR_LEN];
        if (len >= BLE_CHAR_LEN) len = BLE_CHAR_LEN - 1;
        os_mbuf_copydata(ctxt->om, 0, len, rx);
        rx[len] = '\0';
        ESP_LOGI("BLE_RX", "conn=%d: %s", ch, rx);
        return 0;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        os_mbuf_append(ctxt->om, "OK", 2);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = BLE_UUID16_DECLARE(0xFF00),
      .characteristics = (struct ble_gatt_chr_def[]) {
          { .uuid       = BLE_UUID16_DECLARE(0xFF01),
            .access_cb  = gatt_access_cb,
            .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                          BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &s_notify_handle, },
          { 0 } } },
    { 0 }
};

static void ble_start_advertising(void)
{
    if (s_advertising || s_active_conns >= BLE_MAX_CONN) return;
    struct ble_gap_adv_params adv = {0};
    struct ble_hs_adv_fields f   = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = ble_svc_gap_device_name();
    f.name = (uint8_t *)name; f.name_len = strlen(name); f.name_is_complete = 1;
    static uint16_t svc = 0xFF00;
    f.uuids16 = (ble_uuid16_t[]){ BLE_UUID16_INIT(svc) };
    f.num_uuids16 = 1; f.uuids16_is_complete = 1;
    if (ble_gap_adv_set_fields(&f) != 0) return;
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    if (ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                          &adv, gap_event_cb, NULL) == 0) {
        s_advertising = true;
        ESP_LOGI("BLE", "Advertising started");
    }
}

static int gap_event_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_advertising = false;
        if (ev->connect.status == 0) {
            for (int i = 0; i < BLE_MAX_CONN; i++) {
                if (!s_conns[i].in_use) {
                    s_conns[i] = (ble_conn_t){ ev->connect.conn_handle, true, false };
                    s_active_conns++;
                    ble_gattc_exchange_mtu(ev->connect.conn_handle, NULL, NULL);
                    break;
                }
            }
        }
        if (s_active_conns < BLE_MAX_CONN) ble_start_advertising();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        s_advertising = false;
        for (int i = 0; i < BLE_MAX_CONN; i++) {
            if (s_conns[i].in_use &&
                s_conns[i].handle == ev->disconnect.conn.conn_handle) {
                s_conns[i].in_use = false; s_active_conns--;
                break;
            }
        }
        if (s_active_conns < BLE_MAX_CONN) ble_start_advertising();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        for (int i = 0; i < BLE_MAX_CONN; i++) {
            if (s_conns[i].in_use &&
                s_conns[i].handle == ev->subscribe.conn_handle) {
                s_conns[i].notify_en = ev->subscribe.cur_notify;
                break;
            }
        }
        break;
    default: break;
    }
    return 0;
}

static void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ble_svc_gap_device_name_set(s_device_name);   /* uses name from flash */
    ble_gatts_count_cfg(s_gatt_svcs);
    ble_gatts_add_svcs(s_gatt_svcs);
    ble_gatts_start();
    ble_start_advertising();
}

static void ble_host_task(void *p)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init_internal(void)
{
    memset(s_conns, 0, sizeof(s_conns));
    nimble_port_init();
    ble_svc_gap_init(); ble_svc_gatt_init();
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
}

void streaming_ble_send(const char *json_str)
{
    if (!json_str || s_notify_handle == 0) return;
    size_t len = strlen(json_str);
    for (int i = 0; i < BLE_MAX_CONN; i++) {
        if (!s_conns[i].in_use || !s_conns[i].notify_en) continue;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(json_str, (uint16_t)len);
        if (om) ble_gatts_notify_custom(s_conns[i].handle, s_notify_handle, om);
    }
}

/* ════════════════════════════════════════════════════════════════
 *  UDP broadcast tasks
 * ════════════════════════════════════════════════════════════════ */
#define UDP_INTERVAL_MS  1000

static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONN_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = data;
        ESP_LOGI("WIFI", "IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_eg, WIFI_CONN_BIT);
    }
}

/* ════════════════════════════════════════════════════════════════
 *  UDP broadcast — single task, one socket, all ports
 *  All large buffers are static to keep stack usage minimal.
 * ════════════════════════════════════════════════════════════════ */

/* Static buffers — NOT on the task stack */
static char  s_udp_buf[1200];
static char  s_udp_json[JSON_BUF_SIZE];
static data_snapshot_t s_udp_snap;

static void udp_broadcast_task(void *pv)
{
    /* One socket for all ports */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket create failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int bc = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc));

    static const int ports[] = {
        STREAM_UDP_PORT_IMU,
        STREAM_UDP_PORT_HR,
        STREAM_UDP_PORT_ADC,
        STREAM_UDP_PORT_MOTION,
        STREAM_UDP_PORT_VITALS,
        STREAM_UDP_PORT_JSON,
        STREAM_UDP_PORT_STATE,
        STREAM_UDP_PORT_ORIENTATION,
    };
    int num_ports = sizeof(ports) / sizeof(ports[0]);

    ESP_LOGI(TAG, "UDP broadcast task started on %d ports", num_ports);

    while (1) {
        /* Copy snapshot under mutex */
        if (xSemaphoreTake(s_snap_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&s_udp_snap, &s_snap, sizeof(data_snapshot_t));
            strncpy(s_udp_json, s_json_buf, JSON_BUF_SIZE - 1);
            s_udp_json[JSON_BUF_SIZE - 1] = '\0';
            xSemaphoreGive(s_snap_mutex);
        }

        /* Send on each port in turn */
        for (int i = 0; i < num_ports; i++) {
            int port = ports[i];
            int len  = 0;

            switch (port) {
            case STREAM_UDP_PORT_IMU:
                len = snprintf(s_udp_buf, sizeof(s_udp_buf),
                    "ax=%.3f,ay=%.3f,az=%.3f,"
                    "gx=%.3f,gy=%.3f,gz=%.3f,"
                    "mx=%.3f,my=%.3f,mz=%.3f",
                    s_udp_snap.accel_x, s_udp_snap.accel_y, s_udp_snap.accel_z,
                    s_udp_snap.gyro_x,  s_udp_snap.gyro_y,  s_udp_snap.gyro_z,
                    0.0f, 0.0f, 0.0f);
                break;
            case STREAM_UDP_PORT_HR:
                len = snprintf(s_udp_buf, sizeof(s_udp_buf),
                    "HR=%lu,HRV=%.2f,rawIR=%lu",
                    (unsigned long)s_udp_snap.heart_rate,
                    (double)s_udp_snap.hrv,
                    (unsigned long)s_udp_snap.raw_ir);
                break;
            case STREAM_UDP_PORT_ADC:
                len = snprintf(s_udp_buf, sizeof(s_udp_buf),
                    "body=%.2f,amb=%.2f,batt_pct=%u",
                    s_udp_snap.body_temp,
                    s_udp_snap.ambient_temp,
                    s_udp_snap.battery_pct);
                break;
            case STREAM_UDP_PORT_MOTION:
                len = snprintf(s_udp_buf, sizeof(s_udp_buf),
                    "steps=%lu,stepHz=%.2f,var=%.3f",
                    (unsigned long)s_udp_snap.total_steps,
                    s_udp_snap.step_frequency,
                    s_udp_snap.acc_variance);
                break;
            case STREAM_UDP_PORT_VITALS:
                len = snprintf(s_udp_buf, sizeof(s_udp_buf),
                    "dev=%s,HR=%lu,HRV=%.2f,rawIR=%lu,RR=%u,TEMP=%.2f,bat=%u",
                    s_device_name,
                    (unsigned long)s_udp_snap.heart_rate,
                    (double)s_udp_snap.hrv,
                    (unsigned long)s_udp_snap.raw_ir,
                    s_udp_snap.respiration_rate,
                    s_udp_snap.body_temp,
                    s_udp_snap.battery_pct);
                break;
            case STREAM_UDP_PORT_JSON:
                strncpy(s_udp_buf, s_udp_json, sizeof(s_udp_buf) - 1);
                s_udp_buf[sizeof(s_udp_buf) - 1] = '\0';
                len = strlen(s_udp_buf);
                break;
            case STREAM_UDP_PORT_STATE:
                len = snprintf(s_udp_buf, sizeof(s_udp_buf),
                    "ACT=%u,beh=%s,bat=%u%%",
                    s_udp_snap.activity_state,
                    s_udp_snap.behaviour,
                    s_udp_snap.battery_pct);
                break;
            case STREAM_UDP_PORT_ORIENTATION:
                len = snprintf(s_udp_buf, sizeof(s_udp_buf),
                    "roll=%.3f,pitch=%.3f,yaw=%.3f",
                    s_udp_snap.roll,
                    s_udp_snap.pitch,
                    s_udp_snap.yaw);
                break;
            default:
                len = 0;
                break;
            }

            if (len > 0) {
                struct sockaddr_in dest = {
                    .sin_family      = AF_INET,
                    .sin_port        = htons(port),
                    .sin_addr.s_addr = inet_addr("255.255.255.255"),
                };
                sendto(sock, s_udp_buf, len, 0,
                       (struct sockaddr *)&dest, sizeof(dest));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(UDP_INTERVAL_MS));
    }
}

/* ════════════════════════════════════════════════════════════════
 *  UDP Config Receiver  (port 5009)
 *  Listens for:  CMD:SET_WIFI ssid=<s> pass=<p> name=<n>
 *  On receipt: saves to external flash → reboots after 2 s.
 * ════════════════════════════════════════════════════════════════ */
#define STREAM_UDP_PORT_CONFIG  5009

/* Parse key=value from a token string.
 * Searches for "key=" and copies the value (up to next space or end).  */
static bool parse_field(const char *src, const char *key,
                        char *out, size_t out_sz)
{
    const char *p = strstr(src, key);
    if (!p) return false;
    p += strlen(key);
    size_t i = 0;
    while (*p && *p != ' ' && *p != '\r' && *p != '\n' && i < out_sz - 1)
        out[i++] = *p++;
    out[i] = '\0';
    return (i > 0);
}

static void udp_config_task(void *pv)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "config socket failed"); vTaskDelete(NULL); return;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(STREAM_UDP_PORT_CONFIG),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "config bind failed"); close(sock); vTaskDelete(NULL); return;
    }

    ESP_LOGI(TAG, "Config receiver listening on port %d", STREAM_UDP_PORT_CONFIG);

    char buf[256];
    while (1) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&src, &src_len);
        if (n <= 0) continue;
        buf[n] = '\0';

        ESP_LOGI(TAG, "Config RX: %s", buf);

        /* Only accept our specific command prefix */
        if (strncmp(buf, "CMD:SET_WIFI", 12) != 0) continue;

        char ssid[32] = {0}, pass[64] = {0}, name[24] = {0};
        bool ok = parse_field(buf, "ssid=", ssid, sizeof(ssid))
               && parse_field(buf, "name=", name, sizeof(name));
        parse_field(buf, "pass=", pass, sizeof(pass));  /* pass can be empty */

        if (!ok) {
            ESP_LOGW(TAG, "Config parse failed — expected ssid= and name=");
            continue;
        }

        ESP_LOGI(TAG, "Saving config: ssid='%s' name='%s'", ssid, name);

        esp_err_t ret = streaming_save_wifi_config(ssid, pass, name);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Config saved — rebooting in 2 s");
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_restart();
        } else {
            ESP_LOGE(TAG, "Config save failed: %s", esp_err_to_name(ret));
        }
    }
}

static void start_udp_tasks(void)
{
    /* 6 KB stack — lwip sendto + socket internals need headroom */
    BaseType_t ret = xTaskCreate(udp_broadcast_task, "udp_bcast",
                                  6144, NULL, 5, NULL);
    if (ret != pdPASS)
        ESP_LOGE(TAG, "Failed to create UDP broadcast task");
    else
        ESP_LOGI(TAG, "UDP broadcast task started");

    /* Config receiver runs immediately (doesn't need IP — bind is enough) */
    ret = xTaskCreate(udp_config_task, "udp_cfg", 4096, NULL, 6, NULL);
    if (ret != pdPASS)
        ESP_LOGE(TAG, "Failed to create UDP config task");
    else
        ESP_LOGI(TAG, "UDP config receiver started on port %d",
                 STREAM_UDP_PORT_CONFIG);
}

static void wifi_init_internal(void)
{
    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create default event loop only if not already created */
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL, NULL);
    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    strncpy((char *)wc.sta.ssid,     s_wifi_ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, s_wifi_pass,  sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Wait for IP then start UDP tasks */
    xEventGroupWaitBits(s_wifi_eg, WIFI_CONN_BIT, false, true,
                        pdMS_TO_TICKS(30000));
    start_udp_tasks();
}

static void wifi_start_task(void *pv)
{
    wifi_init_internal();
    vTaskDelete(NULL);
}

/* ════════════════════════════════════════════════════════════════
 *  Public init
 * ════════════════════════════════════════════════════════════════ */
void streaming_init(void)
{
    s_snap_mutex = xSemaphoreCreateMutex();

    /* Load WiFi config from external flash — uses fallback defaults if not saved */
    flash_wifi_config_t fcfg = {0};
    if (mw_flash_config_load(&fcfg)) {
        strncpy(s_wifi_ssid,    fcfg.ssid,        sizeof(s_wifi_ssid)    - 1);
        strncpy(s_wifi_pass,    fcfg.password,     sizeof(s_wifi_pass)    - 1);
        strncpy(s_device_name,  fcfg.device_name,  sizeof(s_device_name)  - 1);
        memcpy(s_mac, fcfg.mac, 6);
    } else {
        /* Flash has no config yet — use defaults, will be written when
         * user sends config from the UDP tool */
        esp_efuse_mac_get_default(s_mac);
        ESP_LOGW("STREAM", "Using default WiFi: %s  Device: %s",
                 s_wifi_ssid, s_device_name);
    }

    /* NVS required by both WiFi and BLE */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    ble_init_internal();

    /* WiFi init blocks on DHCP — run in a task so app_main returns
       and lwip TLS is owned by the task that will later call sendto */
    xTaskCreate(wifi_start_task, "wifi_init", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Streaming module ready");
}

/* ── Config save (called by UDP tool) ────────────────────────── */
esp_err_t streaming_save_wifi_config(const char *ssid,
                                     const char *pass,
                                     const char *device_name)
{
    flash_wifi_config_t cfg = {0};
    cfg.magic = FLASH_CONFIG_MAGIC;
    strncpy(cfg.ssid,        ssid,        sizeof(cfg.ssid)        - 1);
    strncpy(cfg.password,    pass,        sizeof(cfg.password)    - 1);
    strncpy(cfg.device_name, device_name, sizeof(cfg.device_name) - 1);
    esp_efuse_mac_get_default(cfg.mac);
    return mw_flash_config_save(&cfg);
}

/* ── Identity helpers ────────────────────────────────────────── */
const char *streaming_get_device_name(void)
{
    return s_device_name;
}

const char *streaming_get_mac_str(void)
{
    static char mac_str[18];
    snprintf(mac_str, sizeof(mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             s_mac[0], s_mac[1], s_mac[2],
             s_mac[3], s_mac[4], s_mac[5]);
    return mac_str;
}
