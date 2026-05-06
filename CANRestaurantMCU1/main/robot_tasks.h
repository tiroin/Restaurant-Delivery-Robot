/**
 * @file    robot_tasks.h
 * @brief   Các FreeRTOS Task của robot — Public API
 *
 * Module này chứa 4 task chính của hệ thống.
 * app_main.c chỉ cần include header này và gọi xTaskCreate().
 *
 * Sử dụng trong app_main:
 * @code
 *   #include "robot_tasks.h"
 *
 *   // Sau khi khởi tạo queue/event group trong can_bus_init():
 *   xTaskCreate(state_machine_task,    "StateMachine",  8192, NULL, 4, NULL);
 *   xTaskCreate(wifi_order_task,       "WiFiOrder",     2048, NULL, 3, NULL);
 *   xTaskCreate(status_broadcast_task, "StatusBcast",   4096, NULL, 2, NULL);
 *   xTaskCreate(watchdog_task,         "Watchdog",      2048, NULL, 6, NULL);
 * @endcode
 *
 * Order từ RainMaker được enqueue qua:
 * @code
 *   order_t order = { ... };
 *   robot_tasks_enqueue_order(&order);   // Gọi từ write_cb
 * @endcode
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "robot_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ════════════════════════════════════════════════════════════════
 *  ORDER QUEUE HANDLE
 *  Defined in robot_tasks.c, extern ở đây để write_cb (app_main.c) push vào.
 * ════════════════════════════════════════════════════════════════ */

/** Queue từ WiFiOrderTask/write_cb → StateMachineTask. Depth=8 */
extern QueueHandle_t xOrderQueue;

/* ════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief Tạo xOrderQueue. Phải gọi TRƯỚC khi tạo bất kỳ task nào.
 * @note  can_bus_init() phải được gọi trước hàm này.
 */
void robot_tasks_init(void);

/**
 * @brief Enqueue một order vào xOrderQueue, thread-safe.
 *        Gọi từ RainMaker write_cb.
 *
 * @param[in]  order  Con trỏ đơn hàng cần enqueue (copy vào queue)
 * @return ESP_OK, ESP_ERR_TIMEOUT nếu queue đầy sau 100ms
 */
esp_err_t robot_tasks_enqueue_order(const order_t *order);

/* ════════════════════════════════════════════════════════════════
 *  FREERTOS TASK FUNCTIONS
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief StateMachineTask — Task trung tâm điều khiển robot.
 *
 *  - Kiểm tra emergency EventGroup ở đầu mỗi vòng lặp
 *  - Xử lý state machine theo thứ tự:
 *    IDLE → GOING_KITCHEN → WAITING_LOAD → GOING_TABLE
 *         → WAITING_UNLOAD → RETURNING_HOME → IDLE
 *  - Gửi lệnh CAN qua can_send_command()
 *  - Cập nhật stats qua robot_stats_xxx()
 *
 * @note  Stack: 8192 bytes, Priority: 4
 */
void state_machine_task(void *pvParam);

/**
 * @brief WiFiOrderTask — Heartbeat + log task.
 *        Order thực sự đến từ write_cb (RainMaker MQTT callback).
 *
 * @note  Stack: 2048 bytes, Priority: 3
 */
void wifi_order_task(void *pvParam);

/**
 * @brief StatusBroadcastTask — Định kỳ push stats lên RainMaker.
 *        Cũng yêu cầu status từ CAN mỗi 10 giây.
 *
 * @note  Stack: 4096 bytes, Priority: 2
 */
void status_broadcast_task(void *pvParam);

/**
 * @brief WatchdogTask — Feed esp_task_wdt mỗi 2 giây.
 *        Task này không được starve — priority cao nhất (6).
 *
 * @note  Stack: 2048 bytes, Priority: 6
 */
void watchdog_task(void *pvParam);

#ifdef __cplusplus
}
#endif
