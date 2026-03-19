#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"

#include "ble.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "BLE"
#define MAX_CONNECTIONS 4
#define MAX_RX_LEN 400  

/* ================= Connection tracking ================= */

typedef struct {
    uint16_t handle;
    bool in_use;
    bool notify_enabled;
} conn_t;

static conn_t conn_list[MAX_CONNECTIONS];

/* ================= Globals ================= */

static uint8_t own_addr_type;
static uint16_t notify_handle;
static bool advertising = false;
static uint8_t active_connections = 0;

/* ================= Forward declarations ================= */

static void ble_sync(void);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_advertising(void);
static int gatt_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg);

/* ================= Notify helpers ================= */

/* Application callback: called by ble.c when a message arrives from a client */
void on_message_received(uint8_t *data, uint16_t len, uint16_t conn_id)
{
    ESP_LOGI(TAG, "APP: message from conn=%d len=%d", conn_id, len);
    // Print ASCII (if printable)
    char buf[CHAR_VALUE_LEN + 1];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, (len < CHAR_VALUE_LEN) ? len : (CHAR_VALUE_LEN - 1));
    ESP_LOGI(TAG, "APP: payload: %s", buf);
 
    // Example: echo back to the same client (will only send if client enabled notifications)
    char reply[] = "Echo: received";
    send_message_to_conn(conn_id, (uint8_t*)reply, strlen(reply));
}

void send_message_to_conn(uint16_t conn_handle, uint8_t *data, uint16_t length)
{
    if (notify_handle == 0 || length == 0) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
    if (!om) {
        ESP_LOGE(TAG, "mbuf alloc failed");
        return;
    }

    int rc = ble_gatts_notify_custom(conn_handle, notify_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Notify failed conn=%d rc=%d", conn_handle, rc);
    }
}

void send_message_all(uint8_t *data, uint16_t length)
{
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (conn_list[i].in_use && conn_list[i].notify_enabled) {
            send_message_to_conn(conn_list[i].handle, data, length);
            printf("sent data to connection : %d\n", i);
        }
    }
}

/* ================= GATT Service ================= */

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFF00),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0xFF01),
                .access_cb = gatt_access_cb,
                .flags =
                    BLE_GATT_CHR_F_READ |
                    BLE_GATT_CHR_F_WRITE |
                    BLE_GATT_CHR_F_WRITE_NO_RSP |
                    BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &notify_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

/* ================= GATT Access ================= */

static int gatt_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    static uint8_t rx_buf[MAX_RX_LEN];   // ⭐ no stack overflow

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {

        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= MAX_RX_LEN) len = MAX_RX_LEN - 1;

        os_mbuf_copydata(ctxt->om, 0, len, rx_buf);
        rx_buf[len] = '\0';

        ESP_LOGI(TAG, "RX from phone conn=%d: %s", conn_handle, rx_buf);

        on_message_received(rx_buf, len, conn_handle);
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const char *msg = "BLE_OK";
        os_mbuf_append(ctxt->om, msg, strlen(msg));
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ================= Advertising ================= */

static void start_advertising(void)
{
    if (advertising || active_connections >= MAX_CONNECTIONS) return;

    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    static uint16_t svc_uuid = 0xFF00;
    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(svc_uuid),
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    if (ble_gap_adv_set_fields(&fields) != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed");
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    if (ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                          &adv_params, gap_event_cb, NULL) == 0) {
        advertising = true;
        ESP_LOGI(TAG, "Advertising started");
    }
}

/* ================= GAP Events ================= */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT: {
        // A device connected
        advertising = false;  // stop any temporary flag

        if (event->connect.status == 0) {
            // Add to connection list
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (!conn_list[i].in_use) {
                    conn_list[i] = (conn_t){
                        .handle = event->connect.conn_handle,
                        .in_use = true,
                        .notify_enabled = false
                    };
                    active_connections++;
                    ESP_LOGI(TAG, "Connected handle=%d slot=%d",
                             event->connect.conn_handle, i);
                    break;
                }
            }

            // Exchange MTU (optional but recommended)
            ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
        } else {
            ESP_LOGW(TAG, "Connection failed status=%d", event->connect.status);
        }

        // ✅ Only restart advertising if we still have free slots
        if (active_connections < MAX_CONNECTIONS && !advertising) {
            start_advertising();
        }

        break;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        advertising = false;

        // Remove connection from list
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (conn_list[i].in_use &&
                conn_list[i].handle == event->disconnect.conn.conn_handle) {

                conn_list[i].in_use = false;
                conn_list[i].notify_enabled = false;
                active_connections--;

                ESP_LOGI(TAG, "Disconnected handle=%d",
                         event->disconnect.conn.conn_handle);
                break;
            }
        }

        // ✅ Restart advertising if we have room for new connections
        if (active_connections < MAX_CONNECTIONS && !advertising) {
            start_advertising();
        }

        break;
    }

    case BLE_GAP_EVENT_SUBSCRIBE: {
        // Update notify status for the correct connection
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (conn_list[i].in_use &&
                conn_list[i].handle == event->subscribe.conn_handle) {

                conn_list[i].notify_enabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "Subscribe conn=%d notify=%d",
                         event->subscribe.conn_handle,
                         event->subscribe.cur_notify);
                break;
            }
        }
        break;
    }

    default:
        break;
    }

    return 0;
}

/* ================= BLE Sync ================= */

static void ble_sync(void)
{
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) return;

    ble_svc_gap_device_name_set("NECKBAND_BLE");

    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_gatts_start();

    start_advertising();
}

/* ================= Host Task ================= */

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ================= Init ================= */

void ble_init(void)
{
    memset(conn_list, 0, sizeof(conn_list));
    active_connections = 0;

    nvs_flash_init();
    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_hs_cfg.sync_cb = ble_sync;

    nimble_port_freertos_init(host_task);
}
