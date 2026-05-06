/**
 * @file    robot_tasks.c
 * @brief   Các FreeRTOS Task — Implementation
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/twai.h"

#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_utils.h"

#include "robot_tasks.h"
#include "robot_stats.h"
#include "can_bus.h"

static const char *TAG_SM     = "StateMachine";
static const char *TAG_ORDER  = "WiFiOrder";
static const char *TAG_STATUS = "StatusBroadcast";
static const char *TAG_WDT    = "Watchdog";

/* ─── Order Queue (extern trong robot_tasks.h) ─── */
QueueHandle_t xOrderQueue = NULL;

/* ─── RainMaker device handles (extern trong app_rmaker.h) ─── */
extern esp_rmaker_device_t *robot_device;
extern esp_rmaker_device_t *stats_device;

/* Timeout tại mỗi trạng thái */
#define TIMEOUT_GOING_MS            30000U
#define TIMEOUT_WAITING_LOAD_MS     60000U
#define TIMEOUT_WAITING_UNLOAD_MS   30000U

/* ════════════════════════════════════════════════════════════
 *  robot_tasks_init
 * ════════════════════════════════════════════════════════════ */
void robot_tasks_init(void)
{
    xOrderQueue = xQueueCreate(8, sizeof(order_t));
    configASSERT(xOrderQueue);
    ESP_LOGI(TAG_ORDER, "xOrderQueue created (depth=8)");
}

/* ════════════════════════════════════════════════════════════
 *  robot_tasks_enqueue_order
 * ════════════════════════════════════════════════════════════ */
esp_err_t robot_tasks_enqueue_order(const order_t *order)
{
    if (!order) return ESP_ERR_INVALID_ARG;

    if (xQueueSend(xOrderQueue, order, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG_ORDER, "xOrderQueue full — order #%lu dropped",
                 order->order_id);
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG_ORDER, "Order #%lu enqueued (table_cp=%d, items=%d)",
             order->order_id, order->table_checkpoint_id, order->item_count);
    return ESP_OK;
}

/* ════════════════════════════════════════════════════════════
 *  Internal helper: gửi lệnh CAN đơn giản
 * ════════════════════════════════════════════════════════════ */
static inline void sm_move_to(uint8_t checkpoint_id)
{
    uint8_t d[1] = { checkpoint_id };
    can_send_command(CAN_ID_MOVE_TO_CP, d, 1);
}

static inline void sm_emergency_stop(void)
{
    can_send_command(CAN_ID_STOP_NOW, NULL, 0);
}

static inline void sm_return_home(void)
{
    can_send_command(CAN_ID_RETURN_HOME, NULL, 0);
}

/* ════════════════════════════════════════════════════════════
 *  StateMachineTask
 *  Priority: 4  |  Stack: 8192
 * ════════════════════════════════════════════════════════════ */
void state_machine_task(void *pvParam)
{
    (void)pvParam;
    ESP_LOGI(TAG_SM, "StateMachineTask started");

    order_t        current_order  = {0};
    bool           has_order      = false;
    twai_message_t rx_frame       = {0};
    TickType_t     state_tick     = 0;

    /* snapshot stats để đọc state hiện tại */
    robot_stats_t  snap;

    for (;;) {
        /* ── 1. KIỂM TRA EMERGENCY ĐẦU MỖI VÒNG LẶP ──────── */
        EventBits_t em = xEventGroupGetBits(xEmergencyGroup);

        if (em & (EMERGENCY_STOP_BIT | OBSTACLE_BIT)) {
            robot_stats_snapshot(&snap);

            if (snap.current_state != ROBOT_STATE_EMERGENCY) {
                /* Lần đầu phát hiện emergency */
                sm_emergency_stop();
                robot_stats_set_state(ROBOT_STATE_EMERGENCY);
                robot_stats_inc_emergency();

                if (em & OBSTACLE_BIT) {
                    robot_stats_inc_obstacle();
                    robot_stats_set_error("Obstacle at cp=%d",
                                          snap.current_checkpoint);
                    esp_rmaker_raise_alert("Obstacle detected — robot stopped!");
                } else {
                    robot_stats_set_error("Emergency stop at cp=%d",
                                          snap.current_checkpoint);
                    esp_rmaker_raise_alert("Emergency stop triggered!");
                }
            }
            /* Đang trong EMERGENCY: chờ app clear bit rồi mới tiếp tục */
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* ── 2. STATE MACHINE ──────────────────────────────── */
        robot_stats_snapshot(&snap);

        switch (snap.current_state) {

        /* ════ IDLE ════ */
        case ROBOT_STATE_IDLE:
        {
            /*
             * Block tối đa 500ms. Nếu có order → chuyển trạng thái.
             * Dùng timeout 500ms thay vì portMAX_DELAY để vòng lặp
             * vẫn kiểm tra emergency kịp thời.
             */
            if (xQueueReceive(xOrderQueue, &current_order,
                              pdMS_TO_TICKS(500)) == pdTRUE) {

                ESP_LOGI(TAG_SM, "Order #%lu received — table_cp=%d  items=%d  urgent=%d",
                         current_order.order_id,
                         current_order.table_checkpoint_id,
                         current_order.item_count,
                         current_order.urgent);

                for (int i = 0; i < current_order.item_count; i++) {
                    ESP_LOGI(TAG_SM, "  [%d] %s", i + 1, current_order.items[i]);
                }

                has_order  = true;
                state_tick = xTaskGetTickCount();
                robot_stats_set_state(ROBOT_STATE_GOING_KITCHEN);
                sm_move_to(CHECKPOINT_ID_KITCHEN);

                ESP_LOGI(TAG_SM, "→ GOING_KITCHEN (cp=%d)", CHECKPOINT_ID_KITCHEN);
            }
            break;
        }

        /* ════ GOING_KITCHEN ════ */
        case ROBOT_STATE_GOING_KITCHEN:
        {
            if (xQueueReceive(xCANRxQueue, &rx_frame,
                              pdMS_TO_TICKS(100)) == pdTRUE) {

                if (rx_frame.identifier == CAN_ID_ARRIVED_CP &&
                    rx_frame.data[0] == CHECKPOINT_ID_KITCHEN) {

                    ESP_LOGI(TAG_SM, "Arrived at Kitchen");
                    robot_stats_set_checkpoint(CHECKPOINT_ID_KITCHEN);
                    robot_stats_set_state(ROBOT_STATE_WAITING_LOAD);
                    state_tick = xTaskGetTickCount();

                    char msg[64];
                    snprintf(msg, sizeof(msg), "Robot tại bếp — đơn #%lu",
                             current_order.order_id);
                    esp_rmaker_raise_alert(msg);

                } else if (rx_frame.identifier == CAN_ID_MOTION_STATUS) {
                    robot_stats_set_battery(rx_frame.data[1]);
                }
            }

            if (xTaskGetTickCount() - state_tick >
                    pdMS_TO_TICKS(TIMEOUT_GOING_MS)) {
                robot_stats_set_error("Timeout going to Kitchen");
                robot_stats_set_state(ROBOT_STATE_ERROR);
            }
            break;
        }

        /* ════ WAITING_LOAD ════ */
        case ROBOT_STATE_WAITING_LOAD:
        {
            /*
             * Bếp xác nhận đặt món bằng cách nhấn nút vật lý trên robot.
             * Motion MCU local gửi CAN_ID_MOTION_STATUS với status=IDLE
             * để báo hiệu "đã xong".
             * Nếu không có tín hiệu sau TIMEOUT_WAITING_LOAD_MS → tiếp tục.
             */
            bool load_confirmed = false;

            if (xQueueReceive(xCANRxQueue, &rx_frame,
                              pdMS_TO_TICKS(200)) == pdTRUE) {
                if (rx_frame.identifier == CAN_ID_MOTION_STATUS &&
                    rx_frame.data[0] == MOTION_STATUS_IDLE) {
                    load_confirmed = true;
                    ESP_LOGI(TAG_SM, "Kitchen confirmed load");
                }
            }

            bool timed_out = (xTaskGetTickCount() - state_tick >
                              pdMS_TO_TICKS(TIMEOUT_WAITING_LOAD_MS));
            if (timed_out) {
                ESP_LOGW(TAG_SM, "Load timeout — proceeding anyway");
            }

            if (load_confirmed || timed_out) {
                state_tick = xTaskGetTickCount();
                robot_stats_set_state(ROBOT_STATE_GOING_TABLE);
                sm_move_to(current_order.table_checkpoint_id);
                ESP_LOGI(TAG_SM, "→ GOING_TABLE (cp=%d)",
                         current_order.table_checkpoint_id);
            }
            break;
        }

        /* ════ GOING_TABLE ════ */
        case ROBOT_STATE_GOING_TABLE:
        {
            if (xQueueReceive(xCANRxQueue, &rx_frame,
                              pdMS_TO_TICKS(100)) == pdTRUE) {

                if (rx_frame.identifier == CAN_ID_ARRIVED_CP &&
                    rx_frame.data[0] == current_order.table_checkpoint_id) {

                    ESP_LOGI(TAG_SM, "Arrived at Table cp=%d",
                             current_order.table_checkpoint_id);
                    robot_stats_set_checkpoint(current_order.table_checkpoint_id);
                    robot_stats_set_state(ROBOT_STATE_WAITING_UNLOAD);
                    state_tick = xTaskGetTickCount();

                    char msg[64];
                    snprintf(msg, sizeof(msg), "Món đã đến bàn — đơn #%lu",
                             current_order.order_id);
                    esp_rmaker_raise_alert(msg);

                } else if (rx_frame.identifier == CAN_ID_MOTION_STATUS) {
                    robot_stats_set_battery(rx_frame.data[1]);
                }
            }

            if (xTaskGetTickCount() - state_tick >
                    pdMS_TO_TICKS(TIMEOUT_GOING_MS)) {
                robot_stats_set_error("Timeout going to table cp=%d",
                                      current_order.table_checkpoint_id);
                robot_stats_set_state(ROBOT_STATE_ERROR);
            }
            break;
        }

        /* ════ WAITING_UNLOAD ════ */
        case ROBOT_STATE_WAITING_UNLOAD:
        {
            /* Timeout 30s → khách đã lấy (hoặc bỏ qua) */
            if (xTaskGetTickCount() - state_tick >
                    pdMS_TO_TICKS(TIMEOUT_WAITING_UNLOAD_MS)) {

                ESP_LOGI(TAG_SM, "Unload timeout — returning home");
                robot_stats_inc_orders();
                robot_stats_inc_trips();
                has_order  = false;
                state_tick = xTaskGetTickCount();
                robot_stats_set_state(ROBOT_STATE_RETURNING_HOME);
                sm_return_home();
                ESP_LOGI(TAG_SM, "→ RETURNING_HOME");
            }
            break;
        }

        /* ════ RETURNING_HOME ════ */
        case ROBOT_STATE_RETURNING_HOME:
        {
            if (xQueueReceive(xCANRxQueue, &rx_frame,
                              pdMS_TO_TICKS(100)) == pdTRUE) {

                if (rx_frame.identifier == CAN_ID_ARRIVED_CP &&
                    rx_frame.data[0] == CHECKPOINT_ID_HOME) {

                    ESP_LOGI(TAG_SM, "Robot returned home — ready");
                    robot_stats_set_checkpoint(CHECKPOINT_ID_HOME);
                    robot_stats_set_state(ROBOT_STATE_IDLE);
                }
            }

            if (xTaskGetTickCount() - state_tick >
                    pdMS_TO_TICKS(TIMEOUT_GOING_MS)) {
                robot_stats_set_error("Timeout returning home");
                robot_stats_set_state(ROBOT_STATE_ERROR);
            }
            break;
        }

        /* ════ EMERGENCY ════ */
        case ROBOT_STATE_EMERGENCY:
            /*
             * Thoát EMERGENCY: app gọi clear_emergency_cb()
             * → xEventGroupClearBits → robot_stats_set_state(IDLE)
             * → vòng lặp tiếp theo sẽ không vào case này nữa.
             */
            vTaskDelay(pdMS_TO_TICKS(500));
            break;

        /* ════ ERROR ════ */
        case ROBOT_STATE_ERROR:
        {
            robot_stats_snapshot(&snap);
            ESP_LOGE(TAG_SM, "ERROR: %s — retry in 5s", snap.last_error);
            esp_rmaker_raise_alert(snap.last_error);
            vTaskDelay(pdMS_TO_TICKS(5000));
            sm_return_home();
            state_tick = xTaskGetTickCount();
            robot_stats_set_state(ROBOT_STATE_RETURNING_HOME);
            break;
        }

        default:
            robot_stats_set_state(ROBOT_STATE_IDLE);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ════════════════════════════════════════════════════════════
 *  WiFiOrderTask
 *  Priority: 3  |  Stack: 2048
 * ════════════════════════════════════════════════════════════ */
void wifi_order_task(void *pvParam)
{
    (void)pvParam;
    ESP_LOGI(TAG_ORDER, "WiFiOrderTask started");
    robot_stats_t snap;
    TickType_t last_log = xTaskGetTickCount();

    for (;;) {
        if (xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(30000)) {
            last_log = xTaskGetTickCount();
            robot_stats_snapshot(&snap);
            ESP_LOGI(TAG_ORDER, "Heartbeat — state=%s  orders_waiting=%d  served=%lu",
                     robot_state_to_str(snap.current_state),
                     (int)uxQueueMessagesWaiting(xOrderQueue),
                     snap.total_orders_served);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ════════════════════════════════════════════════════════════
 *  StatusBroadcastTask
 *  Priority: 2  |  Stack: 4096
 * ════════════════════════════════════════════════════════════ */
void status_broadcast_task(void *pvParam)
{
    (void)pvParam;
    ESP_LOGI(TAG_STATUS, "StatusBroadcastTask started");

    robot_stats_t snap;
    TickType_t    last_rmaker = 0;
    TickType_t    last_can    = 0;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* Push lên RainMaker mỗi 5 giây */
        if (now - last_rmaker >= pdMS_TO_TICKS(5000)) {
            last_rmaker = now;
            robot_stats_snapshot(&snap);

            bool time_synced = esp_rmaker_time_check();

            if (robot_device) {
                esp_rmaker_param_t *state_param = esp_rmaker_device_get_param_by_name(robot_device, "State");
                if (state_param) {
                    if (time_synced) {
                        esp_rmaker_param_update_and_report(
                            state_param,
                            esp_rmaker_str(robot_state_to_str(snap.current_state)));
                    }
                }

                esp_rmaker_param_update_and_report(
                    esp_rmaker_device_get_param_by_name(robot_device, "Checkpoint"),
                    esp_rmaker_int(snap.current_checkpoint));

                esp_rmaker_param_update_and_report(
                    esp_rmaker_device_get_param_by_name(robot_device, "Battery"),
                    esp_rmaker_int(snap.battery_percent));
            }

            if (stats_device) {
                esp_rmaker_param_t *orders_param = esp_rmaker_device_get_param_by_name(stats_device, "OrdersServed");
                if (orders_param) {
                    if (time_synced) {
                        esp_rmaker_param_update_and_report(
                            orders_param,
                            esp_rmaker_int(snap.total_orders_served));
                    }
                }

                esp_rmaker_param_update_and_report(
                    esp_rmaker_device_get_param_by_name(stats_device, "EmergencyStops"),
                    esp_rmaker_int(snap.emergency_stops));

                esp_rmaker_param_update_and_report(
                    esp_rmaker_device_get_param_by_name(stats_device, "ObstacleDetections"),
                    esp_rmaker_int(snap.obstacle_detections));
            }

            ESP_LOGD(TAG_STATUS, "Broadcast: state=%s  bat=%d%%  served=%lu",
                     robot_state_to_str(snap.current_state),
                     snap.battery_percent,
                     snap.total_orders_served);
        }

        /* Yêu cầu status từ Motion MCU mỗi 10 giây */
        if (now - last_can >= pdMS_TO_TICKS(10000)) {
            last_can = now;
            can_send_command(CAN_ID_REQ_STATUS, NULL, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ════════════════════════════════════════════════════════════
 *  WatchdogTask
 *  Priority: 6  |  Stack: 2048
 * ════════════════════════════════════════════════════════════ */
void watchdog_task(void *pvParam)
{
    (void)pvParam;
    ESP_LOGI(TAG_WDT, "WatchdogTask started — subscribing to TWDT");

    esp_task_wdt_add(NULL); /* Đăng ký task này với Task WDT */

    robot_stats_t snap;
    TickType_t    last_log = xTaskGetTickCount();

    for (;;) {
        esp_task_wdt_reset();

        if (xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(30000)) {
            last_log = xTaskGetTickCount();
            robot_stats_snapshot(&snap);
            ESP_LOGI(TAG_WDT, "WDT alive — state=%s  bat=%d%%",
                     robot_state_to_str(snap.current_state),
                     snap.battery_percent);
        }

        /* 2s < TWDT timeout (thường là 5s) */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
