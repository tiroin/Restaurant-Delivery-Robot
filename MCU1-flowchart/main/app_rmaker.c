/**
 * @file    app_rmaker.c
 * @brief   ESP-RainMaker initialization và write callback — Implementation
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_standard_devices.h"
#include "esp_rmaker_schedule.h"
#include "esp_rmaker_scenes.h"
#include "esp_rmaker_ota.h"
#include "esp_rmaker_common_events.h"
#include "app_insights.h"

#include "app_rmaker.h"
#include "robot_tasks.h"
#include "robot_stats.h"
#include "can_bus.h"
#include "checkpoint_storage.h"

static const char *TAG = "AppRainMaker";

/* ─── Device handles (extern trong app_rmaker.h) ─── */
esp_rmaker_device_t *robot_device = NULL;
esp_rmaker_device_t *stats_device = NULL;

/* ─── Order ID counter (tự tăng) ─── */
static uint32_t s_order_id = 1;

/* ════════════════════════════════════════════════════════════
 *  write_cb — Nhận lệnh từ RainMaker MQTT
 * ════════════════════════════════════════════════════════════ */
static esp_err_t write_cb(const esp_rmaker_device_t *device,
                           const esp_rmaker_param_t  *param,
                           const esp_rmaker_param_val_t val,
                           void *priv,
                           esp_rmaker_write_ctx_t *ctx)
{
    const char *name = esp_rmaker_param_get_name(param);
    ESP_LOGI(TAG, "write_cb: param='%s'", name);

    /* ── Đặt món ─────────────────────────────────────────── */
    if (strcmp(name, "Order") == 0) {
        order_t order = {0};
        order.order_id    = s_order_id++;
        order.timestamp_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

        /* Parse CSV tên món */
        char buf[256];
        strncpy(buf, val.val.s, sizeof(buf) - 1);
        char *tok = strtok(buf, ",");
        while (tok && order.item_count < ORDER_ITEMS_MAX) {
            while (*tok == ' ') tok++; /* trim leading space */
            strncpy(order.items[order.item_count++], tok, ORDER_ITEM_NAME_LEN - 1);
            tok = strtok(NULL, ",");
        }

        /* Lấy TableID từ param riêng */
        esp_rmaker_param_t *p_table =
            esp_rmaker_device_get_param_by_name(device, "TableID");
        if (p_table) {
            order.table_checkpoint_id =
                (uint8_t)esp_rmaker_param_get_val(p_table)->val.i;
        } else {
            order.table_checkpoint_id = 2; /* mặc định bàn 1 */
        }

        /* Lấy Urgent */
        esp_rmaker_param_t *p_urgent =
            esp_rmaker_device_get_param_by_name(device, "Urgent");
        if (p_urgent) {
            order.urgent = esp_rmaker_param_get_val(p_urgent)->val.b;
        }

        /* Enqueue */
        esp_err_t err = robot_tasks_enqueue_order(&order);
        if (err == ESP_OK) {
            char ack[64];
            snprintf(ack, sizeof(ack), "Order #%lu queued (%d items)",
                     order.order_id, order.item_count);
            esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_name(device, "LastOrderStatus"),
                esp_rmaker_str(ack));
        } else {
            esp_rmaker_raise_alert("Order queue full! Vui lòng thử lại.");
        }
    }

    /* ── Xóa Emergency ───────────────────────────────────── */
    else if (strcmp(name, "ClearEmergency") == 0 && val.val.b) {
        ESP_LOGI(TAG, "Clearing emergency — robot → IDLE");
        xEventGroupClearBits(xEmergencyGroup, EMERGENCY_STOP_BIT | OBSTACLE_BIT);
        robot_stats_set_state(ROBOT_STATE_IDLE);
        esp_rmaker_param_update_and_report(param, esp_rmaker_bool(false));
    }

    /* ── Thêm checkpoint ─────────────────────────────────── */
    else if (strcmp(name, "AddCheckpoint") == 0) {
        /*
         * Format: "id:name:x:y"
         * Ví dụ: "5:Ban 5:3500.0:1200.0"
         */
        char buf[128];
        strncpy(buf, val.val.s, sizeof(buf) - 1);

        checkpoint_t cp = { .valid = true };
        char *tok = strtok(buf, ":");
        if (tok) { cp.id    = (uint8_t)atoi(tok);          tok = strtok(NULL, ":"); }
        if (tok) { strncpy(cp.name, tok, CHECKPOINT_NAME_LEN - 1); tok = strtok(NULL, ":"); }
        if (tok) { cp.pos_x = strtof(tok, NULL);            tok = strtok(NULL, ":"); }
        if (tok) { cp.pos_y = strtof(tok, NULL); }

        if (checkpoint_save(&cp) == ESP_OK) {
            /* Đồng bộ vị trí sang Motion MCU */
            uint8_t data[9];
            data[0] = cp.id;
            memcpy(&data[1], &cp.pos_x, 4);
            memcpy(&data[5], &cp.pos_y, 4);
            can_send_command(CAN_ID_SET_CP_POS, data, 9);
            ESP_LOGI(TAG, "Checkpoint id=%d saved + synced to MCU", cp.id);
        }
        /* Reset param về rỗng */
        esp_rmaker_param_update_and_report(param, esp_rmaker_str(""));
    }

    /* ── Xóa checkpoint ──────────────────────────────────── */
    else if (strcmp(name, "DeleteCheckpoint") == 0) {
        checkpoint_delete((uint8_t)val.val.i);
        esp_rmaker_param_update_and_report(param, esp_rmaker_int(0));
    }

    return ESP_OK;
}

/* ════════════════════════════════════════════════════════════
 *  Event handler
 * ════════════════════════════════════════════════════════════ */
static void rmaker_event_handler(void *arg, esp_event_base_t base,
                                  int32_t id, void *data)
{
    if (base == RMAKER_EVENT) {
        switch (id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker initialized");      break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT connected");             break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGW(TAG, "MQTT disconnected");          break;
            default: break;
        }
    }
}

/* ════════════════════════════════════════════════════════════
 *  app_rmaker_init
 * ════════════════════════════════════════════════════════════ */
void app_rmaker_init(void)
{
    ESP_ERROR_CHECK(esp_event_handler_register(
        RMAKER_EVENT, ESP_EVENT_ANY_ID, rmaker_event_handler, NULL));

    /* ── Node ── */
    esp_rmaker_config_t cfg = { .enable_time_sync = true };
    esp_rmaker_node_t *node =
        esp_rmaker_node_init(&cfg, "Robot Restaurant", "Robot");
    configASSERT(node);

    /* ══ Device: Robot (điều khiển + trạng thái) ══ */
    robot_device = esp_rmaker_device_create("Robot", NULL, NULL);
    esp_rmaker_device_add_cb(robot_device, write_cb, NULL);

    /* --- Write params (app → ESP32) --- */
    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("Order", NULL,
            esp_rmaker_str(""),
            PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("TableID", NULL,
            esp_rmaker_int(2),
            PROP_FLAG_READ | PROP_FLAG_WRITE));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("Urgent", NULL,
            esp_rmaker_bool(false),
            PROP_FLAG_READ | PROP_FLAG_WRITE));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("ClearEmergency", NULL,
            esp_rmaker_bool(false),
            PROP_FLAG_READ | PROP_FLAG_WRITE));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("AddCheckpoint", NULL,
            esp_rmaker_str(""),
            PROP_FLAG_READ | PROP_FLAG_WRITE));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("DeleteCheckpoint", NULL,
            esp_rmaker_int(0),
            PROP_FLAG_READ | PROP_FLAG_WRITE));

    /* --- Read params (ESP32 → app) --- */
    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("State", NULL,
            esp_rmaker_str("IDLE"),
            PROP_FLAG_READ | PROP_FLAG_TIME_SERIES));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("Checkpoint", NULL,
            esp_rmaker_int(0),
            PROP_FLAG_READ));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("Battery", NULL,
            esp_rmaker_int(100),
            PROP_FLAG_READ));

    esp_rmaker_device_add_param(robot_device,
        esp_rmaker_param_create("LastOrderStatus", NULL,
            esp_rmaker_str("None"),
            PROP_FLAG_READ));

    esp_rmaker_node_add_device(node, robot_device);

    /* ══ Device: Stats (read-only thống kê) ══ */
    stats_device = esp_rmaker_device_create("Stats", NULL, NULL);

    esp_rmaker_device_add_param(stats_device,
        esp_rmaker_param_create("OrdersServed", NULL,
            esp_rmaker_int(0),
            PROP_FLAG_READ | PROP_FLAG_TIME_SERIES));

    esp_rmaker_device_add_param(stats_device,
        esp_rmaker_param_create("EmergencyStops", NULL,
            esp_rmaker_int(0),
            PROP_FLAG_READ));

    esp_rmaker_device_add_param(stats_device,
        esp_rmaker_param_create("ObstacleDetections", NULL,
            esp_rmaker_int(0),
            PROP_FLAG_READ));

    esp_rmaker_node_add_device(node, stats_device);

    /* ── Enable features ── */
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();
    esp_rmaker_ota_enable_default();
    app_insights_enable();

    esp_rmaker_start();
    ESP_LOGI(TAG, "RainMaker started");
}
