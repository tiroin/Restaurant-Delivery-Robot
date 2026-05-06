/**
 * @file    app_main.c
 * @brief   Robot Restaurant Controller — Entry point
 *
 * app_main chỉ làm đúng 4 việc:
 *   1. Khởi tạo các module (mỗi module tự quản lý nội bộ)
 *   2. Khởi tạo WiFi + RainMaker
 *   3. Tạo FreeRTOS tasks
 *   4. Return (FreeRTOS scheduler tiếp quản)
 *
 * Không có logic nghiệp vụ nào ở đây.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "../../common/rmaker_app_network/app_network.h"
#include "nvs_flash.h"
#include "esp_rmaker_utils.h"
#include <time.h>

#include "checkpoint_storage.h"
#include "can_bus.h"
#include "robot_stats.h"
#include "robot_tasks.h"
#include "app_rmaker.h"

static const char *TAG = "Main";

/* ─────────────────────────────────────────────────────────────
 *  Task configuration table
 *  Tất cả task được tạo ở một chỗ, dễ điều chỉnh priority/stack.
 * ───────────────────────────────────────────────────────────── */
typedef struct {
    TaskFunction_t  func;
    const char     *name;
    uint32_t        stack;
    UBaseType_t     priority;
} task_entry_t;

static const task_entry_t k_tasks[] = {
    /* func                   name              stack   priority */
    { status_broadcast_task,  "StatusBroadcast", 4096,  2 },
    { wifi_order_task,        "WiFiOrder",       2048,  3 },
    { state_machine_task,     "StateMachine",    8192,  4 },
    { can_tx_task,            "CANTx",           4096,  4 },
    { can_rx_task,            "CANRx",           4096,  5 },
    { watchdog_task,          "Watchdog",        2048,  6 },
};

/* ─────────────────────────────────────────────────────────────
 *  app_main
 * ───────────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Robot Restaurant Controller v2.0   ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    /* ── 1. Khởi tạo modules ── */
    ESP_ERROR_CHECK(checkpoint_storage_init());
    checkpoint_print_all();

    robot_stats_init();

    ESP_ERROR_CHECK(can_bus_init());

    robot_tasks_init();

    /* ── 2. WiFi + RainMaker ── */
    /* NVS is required by provisioning (and RainMaker components). */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* Network init must happen before esp_rmaker_node_init(). */
    app_network_init();
    ESP_ERROR_CHECK(app_network_set_custom_mfg_data(
        MGF_DATA_DEVICE_TYPE_SWITCH, MFG_DATA_DEVICE_SUBTYPE_SWITCH));

    /* Start RainMaker first, then bring up network provisioning/connection. */
    app_rmaker_init();
    ESP_ERROR_CHECK(app_network_start(POP_TYPE_NONE));

    /* Time-series reporting needs a valid system time (SNTP). */
    esp_err_t ts_err = esp_rmaker_time_sync_init(NULL);
    if (ts_err != ESP_OK && ts_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Time sync init failed: %s", esp_err_to_name(ts_err));
    }
    ts_err = esp_rmaker_time_wait_for_sync(pdMS_TO_TICKS(15000));
    if (ts_err == ESP_OK) {
        time_t now;
        time(&now);
        ESP_LOGI(TAG, "Time synced (epoch=%ld)", (long)now);
    } else {
        ESP_LOGW(TAG, "Time not synced yet (will retry in background): %s", esp_err_to_name(ts_err));
    }

    /* ── 3. Tạo tasks ── */
    for (int i = 0; i < (int)(sizeof(k_tasks) / sizeof(k_tasks[0])); i++) {
        BaseType_t ret = xTaskCreate(
            k_tasks[i].func,
            k_tasks[i].name,
            k_tasks[i].stack,
            NULL,
            k_tasks[i].priority,
            NULL);

        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task '%s'!", k_tasks[i].name);
        } else {
            ESP_LOGI(TAG, "Task '%s' created (stack=%lu, prio=%d)",
                     k_tasks[i].name,
                     k_tasks[i].stack,
                     (int)k_tasks[i].priority);
        }
    }

    ESP_LOGI(TAG, "All tasks created — scheduler running");
    /* app_main returns: FreeRTOS scheduler tiếp quản */
}
