/**
 * @file    wifi_ap_ws.c
 * @brief   WiFi SoftAP + HTTP/WebSocket server — Implementation
 *
 *  Incoming WS JSON commands are translated into TWAI frames and transmitted
 *  immediately (non-blocking). Outgoing telemetry JSON is pushed by the
 *  caller via wifi_ap_ws_broadcast().
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_ap_ws.h"
#include "robot_types.h"
#include "ble_robot.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/semphr.h"

static const char *TAG = "WSAP";

/* ── AP credentials ─────────────────────────── */
#define AP_SSID         "RestaurantGroup3"
#define AP_PASS         "123123@@@"
#define AP_CHANNEL      6
#define AP_MAX_CONN     10

/* ── Embedded HTML pages ─────────────────────── */
extern const uint8_t index_html_start[]    asm("_binary_index_html_start");
extern const uint8_t index_html_end[]      asm("_binary_index_html_end");
extern const uint8_t customer_html_start[] asm("_binary_customer_html_start");
extern const uint8_t customer_html_end[]   asm("_binary_customer_html_end");

/* ── Order queue ─────────────────────────────── */
#define MAX_ORDERS      20
#define MAX_TABLE_NAME  16
#define MAX_TABLES      20
#define MAX_ITEMS_LEN   200

typedef struct {
    uint32_t id;                    /* monotonic order id */
    char     table[MAX_TABLE_NAME]; /* e.g. "Table 1" */
    char     items[MAX_ITEMS_LEN];  /* comma-joined items */
    uint32_t ts;                    /* xTaskGetTickCount() at submission */
    uint8_t  status;                /* 0=pending, 1=delivered */
} customer_order_t;

static customer_order_t s_orders[MAX_ORDERS];
static int          s_order_count  = 0;
static uint32_t     s_order_next_id = 1;
static SemaphoreHandle_t s_order_mutex = NULL;

/* ── Table list (NVS-backed) ─────────────────── */
#define NVS_NS_TABLES   "tables"
#define NVS_KEY_LIST    "list"

static char s_tables_json[512] = "{\"tables\":[]}"; /* default empty */
static SemaphoreHandle_t s_tables_mutex = NULL;

/* ── Selected tables (customer clicked but not yet ordered) ─── */
static char              s_selected[MAX_TABLES][MAX_TABLE_NAME];
static int               s_selected_count = 0;
static SemaphoreHandle_t s_selected_mutex = NULL;

/* ── Emergency stop flag (set by app_main canRxTask on 0x301/0x303) ─── */
static volatile bool     s_emergency_active = false;

void wifi_ap_ws_set_emergency(bool active)
{
    s_emergency_active = active;
    ESP_LOGW(TAG, "[EMRG] emergency=%d — CAN move/stop %s",
             (int)active, active ? "BLOCKED" : "RESUMED");
}

/* Load table list from NVS into s_tables_json */
static void tables_load_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_TABLES, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(s_tables_json) - 1;
    nvs_get_str(h, NVS_KEY_LIST, s_tables_json, &len);
    nvs_close(h);
}

/* Save current s_tables_json to NVS */
static void tables_save_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_TABLES, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, NVS_KEY_LIST, s_tables_json);
    nvs_commit(h);
    nvs_close(h);
}

/* ── HTTP server handle ──────────────────────── */
static httpd_handle_t s_server = NULL;

/* ══════════════════════════════════════════════
 *  Helper — build and transmit a TWAI frame
 * ══════════════════════════════════════════════ */
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
    } else {
        ESP_LOGI(TAG, "[CAN-TX] 0x%03" PRIx32 " len=%u  %02X %02X %02X %02X %02X",
                 id, len, data?data[0]:0, len>1?data[1]:0,
                 len>2?data[2]:0, len>3?data[3]:0, len>4?data[4]:0);
    }
}

/* ══════════════════════════════════════════════
 *  Minimal JSON field extractor (no external deps)
 *  Finds  "key":value  and returns integer value.
 *  Returns def if key not found or not numeric.
 * ══════════════════════════════════════════════ */
static int json_get_int(const char *json, const char *key, int def)
{
    /* Build search pattern  "key":  */
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(json, pat);
    if (!p) return def;
    p += strlen(pat);
    /* Skip whitespace */
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

/* ══════════════════════════════════════════════
 *  WS message parser → CAN TX
 * ══════════════════════════════════════════════ */
static void handle_ws_message(const char *msg)
{
    ESP_LOGI(TAG, "[WS-RX] %s", msg);
    char cmd[24] = {0};
    if (!json_get_str(msg, "cmd", cmd, sizeof(cmd))) {
        ESP_LOGW(TAG, "[WS-RX] no cmd field");
        return;
    }

    if (strcmp(cmd, "move") == 0) {
        /* {"cmd":"move","dir":0-3,"steps":N,"speed":N} */
        if (s_emergency_active) {
            ESP_LOGW(TAG, "[EMRG] move cmd BLOCKED (emergency active)");
            return;
        }
        uint8_t  dir   = (uint8_t)json_get_int(msg, "dir",   0);
        uint16_t steps = (uint16_t)json_get_int(msg, "steps", 0);
        uint16_t speed = 4000;  /* always CRAWL — 125 steps/s */

        uint8_t data[5] = {
            dir,
            (uint8_t)(steps >> 8), (uint8_t)(steps & 0xFF),
            (uint8_t)(speed >> 8), (uint8_t)(speed & 0xFF),
        };
        can_tx(CAN_ID_MANUAL_MOVE, data, sizeof(data));

    } else if (strcmp(cmd, "stop") == 0) {
        /* {"cmd":"stop"} */
        if (s_emergency_active) {
            ESP_LOGW(TAG, "[EMRG] stop cmd BLOCKED (emergency active)");
            return;
        }
        uint8_t data[5] = {0};
        can_tx(CAN_ID_MANUAL_MOVE, data, sizeof(data));

    } else if (strcmp(cmd, "save_cp") == 0) {
        /* {"cmd":"save_cp","id":0-15} */
        uint8_t data[1] = { (uint8_t)json_get_int(msg, "id", 0) };
        can_tx(CAN_ID_SAVE_CP, data, sizeof(data));

    } else if (strcmp(cmd, "set_mode") == 0) {
        /* {"cmd":"set_mode","mode":0-2} */
        uint8_t data[1] = { (uint8_t)json_get_int(msg, "mode", 0) };
        can_tx(CAN_ID_SET_MODE, data, sizeof(data));

    } else if (strcmp(cmd, "set_tables") == 0) {
        /* {"cmd":"set_tables","tables":["Table 1","Table 2",...]} */
        const char *arr = strstr(msg, "\"tables\":");
        if (arr) {
            arr += strlen("\"tables\":");
            while (*arr == ' ' || *arr == '\t') arr++;
            wifi_ap_ws_set_tables(arr);
        }

    } else {
        ESP_LOGW(TAG, "Unknown WS cmd: %s", cmd);
    }
}

/* ══════════════════════════════════════════════
 *  HTTP GET / — serve customer ordering page
 *  (staff app lives on GitHub Pages, not here)
 * ══════════════════════════════════════════════ */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGW(TAG, ">>>> HTTP GET / from sockfd=%d <<<<", httpd_req_to_sockfd(req));
    size_t len = (size_t)(customer_html_end - customer_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)customer_html_start, (ssize_t)len);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  HTTP GET /customer — serve customer ordering page
 * ══════════════════════════════════════════════ */
static esp_err_t customer_get_handler(httpd_req_t *req)
{
    size_t len = (size_t)(customer_html_end - customer_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)customer_html_start, (ssize_t)len);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  HTTP GET /api/tables — return table list JSON
 * ══════════════════════════════════════════════ */
void wifi_ap_ws_mark_delivered(const char *table)
{
    xSemaphoreTake(s_order_mutex, portMAX_DELAY);
    for (int i = 0; i < s_order_count; i++) {
        if (s_orders[i].status == 0 &&
            strncmp(s_orders[i].table, table, MAX_TABLE_NAME) == 0) {
            s_orders[i].status = 1;
        }
    }
    xSemaphoreGive(s_order_mutex);
    ESP_LOGI(TAG, "Mark delivered: table=%s", table);
}

void wifi_ap_ws_set_tables(const char *tables_array)
{
    /* tables_array is a JSON array like ["T1","T2","T3"] possibly with trailing garbage */
    xSemaphoreTake(s_tables_mutex, portMAX_DELAY);
    snprintf(s_tables_json, sizeof(s_tables_json), "{\"tables\":%s}", tables_array);
    /* Find the closing ] and terminate the JSON object cleanly */
    char *end = strchr(s_tables_json, ']');
    if (end) { end[1] = '}'; end[2] = '\0'; }
    xSemaphoreGive(s_tables_mutex);
    tables_save_nvs();
    ESP_LOGI(TAG, "Tables updated: %s", s_tables_json);
}

static esp_err_t api_tables_get_handler(httpd_req_t *req)
{
    /* Build {"tables":[...],"occupied":[...]} */
    char buf[640];
    int pos = 0;

    /* Copy base tables JSON (already has {"tables":[...]}) */
    xSemaphoreTake(s_tables_mutex, portMAX_DELAY);
    /* Find the closing } to insert occupied array before it */
    char base[512];
    strncpy(base, s_tables_json, sizeof(base) - 1);
    base[sizeof(base) - 1] = '\0';
    xSemaphoreGive(s_tables_mutex);

    /* Strip trailing } to append occupied */
    char *close = strrchr(base, '}');
    if (close) *close = '\0';
    pos = snprintf(buf, sizeof(buf), "%s,\"occupied\":", base);

    /* Collect occupied: pending orders + currently selected */
    char seen[MAX_ORDERS + MAX_TABLES][MAX_TABLE_NAME];
    int  seen_count = 0;
    bool first = true;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "[");

    /* 1. Tables with at least one pending order */
    xSemaphoreTake(s_order_mutex, portMAX_DELAY);
    for (int i = 0; i < s_order_count; i++) {
        if (s_orders[i].status != 0) continue;
        bool dup = false;
        for (int j = 0; j < seen_count; j++) {
            if (strcmp(seen[j], s_orders[i].table) == 0) { dup = true; break; }
        }
        if (dup) continue;
        strncpy(seen[seen_count++], s_orders[i].table, MAX_TABLE_NAME - 1);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%s\"%s\"",
                        first ? "" : ",", s_orders[i].table);
        first = false;
    }
    xSemaphoreGive(s_order_mutex);

    /* 2. Tables currently selected by a customer (not yet ordered) */
    xSemaphoreTake(s_selected_mutex, portMAX_DELAY);
    for (int i = 0; i < s_selected_count; i++) {
        bool dup = false;
        for (int j = 0; j < seen_count; j++) {
            if (strcmp(seen[j], s_selected[i]) == 0) { dup = true; break; }
        }
        if (dup) continue;
        strncpy(seen[seen_count++], s_selected[i], MAX_TABLE_NAME - 1);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%s\"%s\"",
                        first ? "" : ",", s_selected[i]);
        first = false;
    }
    xSemaphoreGive(s_selected_mutex);

    snprintf(buf + pos, sizeof(buf) - pos, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  HTTP GET /api/orders — return pending orders JSON
 * ══════════════════════════════════════════════ */
static esp_err_t api_orders_get_handler(httpd_req_t *req)
{
    char buf[1024];
    int pos = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "{\"orders\":[");

    xSemaphoreTake(s_order_mutex, portMAX_DELAY);
    for (int i = 0; i < s_order_count && pos < (int)sizeof(buf) - 80; i++) {
        if (i > 0) pos += snprintf(buf + pos, sizeof(buf) - pos, ",");
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            "{\"id\":%" PRIu32 ",\"table\":\"%s\",\"items\":\"%s\",\"ts\":%" PRIu32 ",\"status\":%d}",
            s_orders[i].id, s_orders[i].table, s_orders[i].items,
            s_orders[i].ts, s_orders[i].status);
    }
    xSemaphoreGive(s_order_mutex);

    snprintf(buf + pos, sizeof(buf) - pos, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  HTTP POST /api/order — submit an order
 *  Body: {"table":"Table 1","items":"Pho, Spring Roll"}
 * ══════════════════════════════════════════════ */
static esp_err_t api_order_post_handler(httpd_req_t *req)
{
    /* Read body (max 512 bytes) */
    char body[512];
    int  total = req->content_len < (int)sizeof(body) - 1
                  ? req->content_len : (int)sizeof(body) - 1;
    int  recv  = 0;
    while (recv < total) {
        int r = httpd_req_recv(req, body + recv, total - recv);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Read error");
            return ESP_FAIL;
        }
        recv += r;
    }
    body[recv] = '\0';

    /* Parse table and items */
    char table[MAX_TABLE_NAME] = {0};
    char items[MAX_ITEMS_LEN]  = {0};
    json_get_str(body, "table", table, sizeof(table));
    json_get_str(body, "items", items, sizeof(items));

    if (table[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing table");
        return ESP_FAIL;
    }

    /* Enqueue order */
    xSemaphoreTake(s_order_mutex, portMAX_DELAY);
    uint32_t oid = s_order_next_id++;
    if (s_order_count < MAX_ORDERS) {
        customer_order_t *o = &s_orders[s_order_count++];
        o->id     = oid;
        o->ts     = (uint32_t)xTaskGetTickCount();
        o->status = 0;
        strlcpy(o->table, table, sizeof(o->table));
        strlcpy(o->items, items, sizeof(o->items));
    }
    xSemaphoreGive(s_order_mutex);

    /* Remove table from selected list (order now tracks it) */
    xSemaphoreTake(s_selected_mutex, portMAX_DELAY);
    for (int i = 0; i < s_selected_count; i++) {
        if (strncmp(s_selected[i], table, MAX_TABLE_NAME) == 0) {
            for (int j = i; j < s_selected_count - 1; j++)
                memcpy(s_selected[j], s_selected[j + 1], MAX_TABLE_NAME);
            s_selected_count--;
            break;
        }
    }
    xSemaphoreGive(s_selected_mutex);

    /* Notify staff via BLE */
    char notify[320];
    snprintf(notify, sizeof(notify),
        "{\"type\":\"order\",\"id\":%" PRIu32 ",\"table\":\"%s\",\"items\":\"%s\"}",
        oid, table, items);
    ble_robot_notify(notify);
    /* Also broadcast to any connected staff WS client */
    wifi_ap_ws_broadcast(notify);

    /* Respond */
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"id\":%" PRIu32 "}", oid);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    ESP_LOGI(TAG, "[ORDER] #%" PRIu32 " table=%s items=%s", oid, table, items);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  HTTP POST /api/clear — staff lifts occupied state for a table
 *  Body: {"table":"T1"}
 * ══════════════════════════════════════════════ */
static esp_err_t api_clear_post_handler(httpd_req_t *req)
{
    char body[128];
    int total = req->content_len < (int)sizeof(body) - 1
                ? req->content_len : (int)sizeof(body) - 1;
    int recv = 0;
    while (recv < total) {
        int r = httpd_req_recv(req, body + recv, total - recv);
        if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Read error"); return ESP_FAIL; }
        recv += r;
    }
    body[recv] = '\0';
    char table[MAX_TABLE_NAME] = {0};
    json_get_str(body, "table", table, sizeof(table));
    if (table[0] != '\0') {
        wifi_ap_ws_mark_delivered(table);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    ESP_LOGI(TAG, "[CLEAR] table=%s", table);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  HTTP POST /api/select — mark table as selected
 *  Body: {"table":"T1"}
 * ══════════════════════════════════════════════ */
static esp_err_t api_select_post_handler(httpd_req_t *req)
{
    char body[128];
    int total = req->content_len < (int)sizeof(body) - 1
                ? req->content_len : (int)sizeof(body) - 1;
    int recv = 0;
    while (recv < total) {
        int r = httpd_req_recv(req, body + recv, total - recv);
        if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Read error"); return ESP_FAIL; }
        recv += r;
    }
    body[recv] = '\0';
    char table[MAX_TABLE_NAME] = {0};
    json_get_str(body, "table", table, sizeof(table));
    if (table[0] == '\0') { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing table"); return ESP_FAIL; }
    xSemaphoreTake(s_selected_mutex, portMAX_DELAY);
    bool found = false;
    for (int i = 0; i < s_selected_count; i++) {
        if (strncmp(s_selected[i], table, MAX_TABLE_NAME) == 0) { found = true; break; }
    }
    if (!found && s_selected_count < MAX_TABLES) {
        strncpy(s_selected[s_selected_count++], table, MAX_TABLE_NAME - 1);
    }
    xSemaphoreGive(s_selected_mutex);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    ESP_LOGI(TAG, "[SELECT] table=%s", table);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  HTTP POST /api/deselect — release table selection
 *  Body: {"table":"T1"}
 * ══════════════════════════════════════════════ */
static esp_err_t api_deselect_post_handler(httpd_req_t *req)
{
    char body[128];
    int total = req->content_len < (int)sizeof(body) - 1
                ? req->content_len : (int)sizeof(body) - 1;
    int recv = 0;
    while (recv < total) {
        int r = httpd_req_recv(req, body + recv, total - recv);
        if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Read error"); return ESP_FAIL; }
        recv += r;
    }
    body[recv] = '\0';
    char table[MAX_TABLE_NAME] = {0};
    json_get_str(body, "table", table, sizeof(table));
    xSemaphoreTake(s_selected_mutex, portMAX_DELAY);
    for (int i = 0; i < s_selected_count; i++) {
        if (strncmp(s_selected[i], table, MAX_TABLE_NAME) == 0) {
            for (int j = i; j < s_selected_count - 1; j++)
                memcpy(s_selected[j], s_selected[j + 1], MAX_TABLE_NAME);
            s_selected_count--;
            break;
        }
    }
    xSemaphoreGive(s_selected_mutex);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    ESP_LOGI(TAG, "[DESELECT] table=%s", table);
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  WebSocket handler
 * ══════════════════════════════════════════════ */
static esp_err_t ws_handler(httpd_req_t *req)
{
    /* Upgrade handshake (HTTP GET that gets promoted to WS) */
    if (req->method == HTTP_GET) {
        ESP_LOGW(TAG, ">>>> WS CLIENT CONNECTED fd=%d <<<<",
                 httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    /* Receive frame header first (len=0 gives us the frame metadata) */
    httpd_ws_frame_t frame = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    /* Ignore non-text frames (ping/pong/close handled by httpd internally) */
    if (frame.type != HTTPD_WS_TYPE_TEXT || frame.len == 0) {
        return ESP_OK;
    }

    /* Allocate buffer and receive payload */
    uint8_t *buf = calloc(1, frame.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;

    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK) {
        buf[frame.len] = '\0';
        handle_ws_message((const char *)buf);
    } else {
        ESP_LOGW(TAG, "ws recv payload err: %s", esp_err_to_name(ret));
    }

    free(buf);
    return ret;
}

/* ══════════════════════════════════════════════
 *  Public API — broadcast to all WS clients
 *
 *  IMPORTANT: httpd_ws_send_frame_async() is NOT safe to call from
 *  arbitrary task context. We must defer the send to the httpd
 *  worker thread via httpd_queue_work(). The JSON string is duplicated
 *  on the heap and freed by the worker after sending.
 * ══════════════════════════════════════════════ */
typedef struct {
    httpd_handle_t hd;
    int            fd;
    char          *payload;
    size_t         len;
} ws_async_t;

static void ws_async_send(void *arg)
{
    ws_async_t *a = (ws_async_t *)arg;
    httpd_ws_frame_t frame = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)a->payload,
        .len     = a->len,
        .final   = true,
    };
    httpd_ws_send_frame_async(a->hd, a->fd, &frame);
    free(a->payload);
    free(a);
}

/* Public stats — read by app_main periodic stats task */
volatile uint32_t g_ws_clients   = 0;
volatile uint32_t g_ws_queued    = 0;
volatile uint32_t g_ws_queue_err = 0;
volatile uint32_t g_ws_no_client = 0;
volatile uint32_t g_ws_calls     = 0;

esp_err_t wifi_ap_ws_broadcast(const char *json_str)
{
    g_ws_calls++;
    if (!s_server || !json_str) return ESP_FAIL;

    /* httpd_get_client_list: input n = capacity, output n = count filled */
    size_t n = CONFIG_LWIP_MAX_SOCKETS;  /* safe upper bound */
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    if (httpd_get_client_list(s_server, &n, fds) != ESP_OK || n == 0) {
        g_ws_clients = 0;
        g_ws_no_client++;
        return ESP_OK;
    }

    size_t json_len = strlen(json_str);
    uint32_t ws_count = 0;

    for (size_t i = 0; i < n; i++) {
        if (httpd_ws_get_fd_info(s_server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET)
            continue;
        ws_count++;
        ws_async_t *a = malloc(sizeof(*a));
        if (!a) { g_ws_queue_err++; continue; }
        a->payload = malloc(json_len + 1);
        if (!a->payload) { free(a); g_ws_queue_err++; continue; }
        memcpy(a->payload, json_str, json_len + 1);
        a->hd  = s_server;
        a->fd  = fds[i];
        a->len = json_len;
        if (httpd_queue_work(s_server, ws_async_send, a) != ESP_OK) {
            free(a->payload); free(a);
            g_ws_queue_err++;
        } else {
            g_ws_queued++;
        }
    }
    g_ws_clients = ws_count;
    if (ws_count == 0) g_ws_no_client++;

    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  Internal — start HTTP server
 * ══════════════════════════════════════════════ */
static esp_err_t start_webserver(void)
{
    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets  = 10;
    config.max_uri_handlers  = 10;
    config.stack_size        = 8192;
    config.lru_purge_enable  = true;
    config.uri_match_fn     = httpd_uri_match_wildcard;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config),
                        TAG, "httpd_start failed");

    static const httpd_uri_t root_uri = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = root_get_handler,
    };
    httpd_register_uri_handler(s_server, &root_uri);

    static const httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .is_websocket = true,
    };
    httpd_register_uri_handler(s_server, &ws_uri);

    static const httpd_uri_t customer_uri = {
        .uri     = "/customer",
        .method  = HTTP_GET,
        .handler = customer_get_handler,
    };
    httpd_register_uri_handler(s_server, &customer_uri);

    static const httpd_uri_t api_tables_uri = {
        .uri     = "/api/tables",
        .method  = HTTP_GET,
        .handler = api_tables_get_handler,
    };
    httpd_register_uri_handler(s_server, &api_tables_uri);

    static const httpd_uri_t api_orders_uri = {
        .uri     = "/api/orders",
        .method  = HTTP_GET,
        .handler = api_orders_get_handler,
    };
    httpd_register_uri_handler(s_server, &api_orders_uri);

    static const httpd_uri_t api_order_post_uri = {
        .uri     = "/api/order",
        .method  = HTTP_POST,
        .handler = api_order_post_handler,
    };
    httpd_register_uri_handler(s_server, &api_order_post_uri);

    static const httpd_uri_t api_clear_uri = {
        .uri     = "/api/clear",
        .method  = HTTP_POST,
        .handler = api_clear_post_handler,
    };
    httpd_register_uri_handler(s_server, &api_clear_uri);

    static const httpd_uri_t api_select_uri = {
        .uri     = "/api/select",
        .method  = HTTP_POST,
        .handler = api_select_post_handler,
    };
    httpd_register_uri_handler(s_server, &api_select_uri);

    static const httpd_uri_t api_deselect_uri = {
        .uri     = "/api/deselect",
        .method  = HTTP_POST,
        .handler = api_deselect_post_handler,
    };
    httpd_register_uri_handler(s_server, &api_deselect_uri);

    ESP_LOGI(TAG, "HTTP server on :80  WS /ws  Customer /customer  API /api/*");
    return ESP_OK;
}

/* ══════════════════════════════════════════════
 *  WiFi event handler (logging only)
 * ══════════════════════════════════════════════ */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t *e =
                (wifi_event_ap_staconnected_t *)data;
            ESP_LOGW(TAG, ">>>> STA CONNECTED aid=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X <<<<",
                     e->aid, e->mac[0],e->mac[1],e->mac[2],e->mac[3],e->mac[4],e->mac[5]);
        } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t *e =
                (wifi_event_ap_stadisconnected_t *)data;
            ESP_LOGW(TAG, ">>>> STA DISCONNECTED aid=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X <<<<",
                     e->aid, e->mac[0],e->mac[1],e->mac[2],e->mac[3],e->mac[4],e->mac[5]);
        }
    }
}

/* ══════════════════════════════════════════════
 *  Public API — init
 * ══════════════════════════════════════════════ */
esp_err_t wifi_ap_ws_init(void)
{
    /* Init mutexes and load saved table list */
    s_order_mutex   = xSemaphoreCreateMutex();
    s_tables_mutex  = xSemaphoreCreateMutex();
    s_selected_mutex = xSemaphoreCreateMutex();
    tables_load_nvs();

    /* Init TCP/IP stack and default event loop (required before netif/wifi) */
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event_loop_create failed");

    /* Create default AP netif (provides DHCP server + lwIP AP interface) */
    esp_netif_create_default_wifi_ap();

    /* Init Wi-Fi driver */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");

    /* Register event handler for connection logging */
    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            wifi_event_handler, NULL, NULL),
        TAG, "event_handler_register failed");

    /* Configure AP */
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid           = AP_SSID,
            .ssid_len       = (uint8_t)strlen(AP_SSID),
            .password       = AP_PASS,
            .channel        = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP),
                        TAG, "esp_wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg),
                        TAG, "esp_wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(),
                        TAG, "esp_wifi_start failed");

    /* Disable power-save: prevents WiFi radio from sleeping between TX bursts.
       Without this the modem sleeps ~500-800ms between packets, causing exactly
       the large WS gaps observed in debug (all streams drop together). */
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE),
                        TAG, "esp_wifi_set_ps failed");

    ESP_LOGI(TAG, "WiFi AP ready — SSID:\"%s\"  Pass:\"%s\"  IP:192.168.4.1",
             AP_SSID, AP_PASS);

    return start_webserver();
}
