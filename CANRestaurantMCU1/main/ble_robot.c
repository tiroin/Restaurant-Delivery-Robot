/**
 * @file    ble_robot.c
 * @brief   BLE GATT peripheral for robot control using NimBLE (Nordic UART Service)
 *
 *  NUS Service:  6e400001-b5a3-f393-e0a9-e50e24dcca9e
 *  RX (CMD):     6e400002-b5a3-f393-e0a9-e50e24dcca9e  Write/WriteNoRsp  phone→robot
 *  TX (TEL):     6e400003-b5a3-f393-e0a9-e50e24dcca9e  Notify            robot→phone
 *
 *  JSON command protocol (same as old WebSocket):
 *    {"cmd":"move","dir":0,"steps":20}      → CAN 0x110 MANUAL_MOVE
 *    {"cmd":"stop"}                          → CAN 0x110 all-zeros
 *    {"cmd":"save_cp","id":1}               → CAN 0x111 SAVE_CP
 *    {"cmd":"set_mode","mode":1}            → CAN 0x113 SET_MODE
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/twai.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_robot.h"
#include "robot_types.h"
#include "wifi_ap_ws.h"

static const char *TAG = "BLE";

#define DEVICE_NAME "RobotControl"

/* ── NUS UUIDs — 128-bit, bytes in little-endian order for NimBLE ── */
/* 6e400001-b5a3-f393-e0a9-e50e24dcca9e */
static const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* 6e400002-b5a3-f393-e0a9-e50e24dcca9e  (phone→robot Write) */
static const ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

/* 6e400003-b5a3-f393-e0a9-e50e24dcca9e  (robot→phone Notify) */
static const ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_tel_handle   = 0;
static uint16_t s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;

/* ── JSON helpers (same as old wifi_ap_ws.c) ──────────────────────── */

static void can_tx(uint32_t id, const uint8_t *data, uint8_t len)
{
    twai_message_t frame = {
        .identifier       = id,
        .data_length_code = len,
    };
    if (len > 0 && data) {
        memcpy(frame.data, data, len);
    }
    esp_err_t err = twai_transmit(&frame, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_transmit 0x%03" PRIx32 " FAIL: %s",
                 id, esp_err_to_name(err));
    }
}

static int json_get_int(const char *json, const char *key, int def)
{
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(json, pat);
    if (!p) return def;
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') p++;
    int v = def;
    sscanf(p, "%d", &v);
    return v;
}

static int json_get_str(const char *json, const char *key,
                        char *out, size_t out_len)
{
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char *p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    size_t i = 0;
    while (*p && *p != '"' && i < out_len - 1) out[i++] = *p++;
    out[i] = '\0';
    return (int)i;
}

static void handle_command(const char *msg)
{
    ESP_LOGI(TAG, "[BLE-RX] %s", msg);

    char cmd[24] = {0};
    if (!json_get_str(msg, "cmd", cmd, sizeof(cmd))) return;

    if (strcmp(cmd, "move") == 0) {
        uint8_t  dir   = (uint8_t) json_get_int(msg, "dir",   0);
        uint16_t steps = (uint16_t)json_get_int(msg, "steps", 0);
        uint16_t speed = 4000;   /* µs/step — kept at crawl speed */
        uint8_t data[5] = {
            dir,
            (uint8_t)(steps >> 8), (uint8_t)(steps & 0xFF),
            (uint8_t)(speed >> 8), (uint8_t)(speed & 0xFF)
        };
        can_tx(CAN_ID_MANUAL_MOVE, data, sizeof(data));

    } else if (strcmp(cmd, "stop") == 0) {
        uint8_t data[5] = {0};
        can_tx(CAN_ID_MANUAL_MOVE, data, sizeof(data));

    } else if (strcmp(cmd, "save_cp") == 0) {
        uint8_t id = (uint8_t)json_get_int(msg, "id", 0);
        can_tx(CAN_ID_SAVE_CP, &id, 1);

    } else if (strcmp(cmd, "set_mode") == 0) {
        uint8_t mode = (uint8_t)json_get_int(msg, "mode", 0);
        can_tx(CAN_ID_SET_MODE, &mode, 1);

    } else if (strcmp(cmd, "set_tables") == 0) {
        /* Forward table list to WiFi AP module for storage and HTTP serving */
        const char *arr = strstr(msg, "\"tables\":");
        if (arr) {
            arr += strlen("\"tables\":");
            while (*arr == ' ' || *arr == '\t') arr++;
            wifi_ap_ws_set_tables(arr);
        }

    } else if (strcmp(cmd, "order_delivered") == 0) {
        /* Mark all pending orders for this table as delivered */
        char table[32] = {0};
        if (json_get_str(msg, "table", table, sizeof(table))) {
            wifi_ap_ws_mark_delivered(table);
        }
    }
}

/* ── GATT access callbacks ────────────────────────────────────────── */

static int cmd_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0 || len >= 256) return BLE_ATT_ERR_INSUFFICIENT_RES;

    char buf[256];
    uint16_t copied = 0;
    ble_hs_mbuf_to_flat(ctxt->om, buf, len, &copied);
    buf[copied] = '\0';

    handle_command(buf);
    return 0;
}

static int tel_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)ctxt; (void)arg;
    return 0;  /* notify-only; reads return empty */
}

/* ── GATT service table ───────────────────────────────────────────── */

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &NUS_SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {   /* NUS RX — phone writes commands to robot */
                .uuid       = &NUS_RX_UUID.u,
                .access_cb  = cmd_access_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {   /* NUS TX — robot notifies telemetry to phone */
                .uuid       = &NUS_TX_UUID.u,
                .access_cb  = tel_access_cb,
                .val_handle = &s_tel_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },  /* end of characteristics */
        },
    },
    { 0 },  /* end of services */
};

/* ── GAP event handler (forward declaration) ─────────────────────── */
static int gap_event_handler(struct ble_gap_event *event, void *arg);

/* ── Advertising ─────────────────────────────────────────────────── */

static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags           = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name            = (const uint8_t *)DEVICE_NAME;
    fields.name_len        = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as '%s'", DEVICE_NAME);
    }
}

/* ── GAP event handler ───────────────────────────────────────────── */

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected  handle=%d", s_conn_handle);
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(TAG, "BLE connect failed, restarting adv");
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected  reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise();
        break;

    default:
        break;
    }
    return 0;
}

/* ── NimBLE host sync / reset callbacks ──────────────────────────── */

static void ble_app_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    ble_app_advertise();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset  reason=%d", reason);
}

/* ── NimBLE host task ────────────────────────────────────────────── */

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();          /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ── Public API ──────────────────────────────────────────────────── */

esp_err_t ble_robot_init(void)
{
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_count_cfg: %d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_add_svcs: %d", rc);
        return ESP_FAIL;
    }

    ble_svc_gap_device_name_set(DEVICE_NAME);

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE robot init OK — device name '%s'", DEVICE_NAME);
    return ESP_OK;
}

esp_err_t ble_robot_notify(const char *json_str)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return ESP_FAIL;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(json_str, strlen(json_str));
    if (!om) return ESP_ERR_NO_MEM;

    int rc = ble_gatts_notify_custom(s_conn_handle, s_tel_handle, om);
    if (rc != 0) {
        ESP_LOGD(TAG, "notify_custom: %d (client may not have subscribed yet)", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool ble_robot_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}
