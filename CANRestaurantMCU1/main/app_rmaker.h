/**
 * @file    app_rmaker.h
 * @brief   ESP-RainMaker initialization và write callback — Public API
 *
 * Module này đăng ký 2 RainMaker device:
 *   - "Robot"  : điều khiển (Order, TableID, Urgent, ClearEmergency, ...)
 *   - "Stats"  : read-only thống kê (OrdersServed, Battery, ...)
 *
 * Sử dụng:
 * @code
 *   #include "app_rmaker.h"
 *
 *   // Trong app_main, sau esp_event_loop_create_default():
 *   app_rmaker_init();
 *   // Xong — RainMaker tự kết nối sau khi WiFi provisioned.
 * @endcode
 *
 * Các param RainMaker khách hàng/app gửi xuống:
 *
 *  | Param            | Type   | Mô tả                                         |
 *  |------------------|--------|------------------------------------------------|
 *  | Order            | string | CSV tên món, vd "Pho,Ca phe sua"              |
 *  | TableID          | int    | checkpoint_id của bàn cần giao                 |
 *  | Urgent           | bool   | true = ưu tiên cao                             |
 *  | ClearEmergency   | bool   | true = xóa emergency flag, robot về IDLE       |
 *  | AddCheckpoint    | string | "id:name:x:y" để lưu checkpoint mới vào flash  |
 *  | DeleteCheckpoint | int    | id checkpoint cần xóa                          |
 */

#pragma once

#include "esp_err.h"
#include "esp_rmaker_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Robot control device handle (read bởi StatusBroadcastTask) */
extern esp_rmaker_device_t *robot_device;

/** Stats device handle */
extern esp_rmaker_device_t *stats_device;

/**
 * @brief Khởi tạo ESP-RainMaker node, đăng ký device và params,
 *        bật OTA, Schedule, Scenes, rồi gọi esp_rmaker_start().
 *
 * @note  Phải gọi SAU esp_event_loop_create_default() và app_network_start().
 */
void app_rmaker_init(void);

#ifdef __cplusplus
}
#endif
