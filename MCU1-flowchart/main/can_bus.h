/**
 * @file    can_bus.h
 * @brief   CAN Bus Driver + CANTxTask + CANRxTask — Public API
 *
 * Module này quản lý toàn bộ giao tiếp CAN (TWAI) với Motion MCU.
 * Bên ngoài chỉ cần:
 *   1. Gọi can_bus_init() một lần trong app_main
 *   2. Gọi can_send_command() bất cứ đâu để gửi lệnh
 *   3. Đọc xCANRxQueue để nhận phản hồi (làm bởi StateMachineTask)
 *
 * Sử dụng:
 * @code
 *   // app_main.c
 *   can_bus_init();
 *   xTaskCreate(can_tx_task, "CANTx", 4096, NULL, 4, NULL);
 *   xTaskCreate(can_rx_task, "CANRx", 4096, NULL, 5, NULL);
 *
 *   // Bất kỳ nơi nào muốn gửi lệnh di chuyển:
 *   uint8_t data[1] = { checkpoint_id };
 *   can_send_command(CAN_ID_MOVE_TO_CP, data, 1);
 *
 *   // Emergency stop tức thì:
 *   can_send_command(CAN_ID_STOP_NOW, NULL, 0);
 * @endcode
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "robot_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ════════════════════════════════════════════════════════════════
 *  QUEUE & EVENT GROUP HANDLES
 *  Defined in can_bus.c, extern ở đây để các module khác truy cập.
 * ════════════════════════════════════════════════════════════════ */

/** Queue gửi TWAI frame từ StateMachineTask → CANTxTask. Depth=16 */
extern QueueHandle_t xCANTxQueue;

/** Queue nhận TWAI frame từ CANRxTask → StateMachineTask. Depth=32 */
extern QueueHandle_t xCANRxQueue;

/** EventGroup báo emergency. Bit EMERGENCY_STOP_BIT và OBSTACLE_BIT */
extern EventGroupHandle_t xEmergencyGroup;

/** Bit definitions cho xEmergencyGroup */
#define EMERGENCY_STOP_BIT      BIT0
#define OBSTACLE_BIT            BIT1

/* ════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief Khởi tạo TWAI driver, tạo queue và event group.
 * @note  Phải gọi TRƯỚC khi tạo can_tx_task / can_rx_task.
 * @return ESP_OK nếu thành công
 */
esp_err_t can_bus_init(void);

/**
 * @brief Build và enqueue một CAN frame vào xCANTxQueue.
 *        Hàm non-blocking (timeout 100ms nếu queue đầy).
 *
 * @param[in]  can_id  11-bit CAN identifier (dùng CAN_ID_xxx từ robot_types.h)
 * @param[in]  data    Con trỏ data bytes, NULL nếu len=0
 * @param[in]  len     Số byte data [0..8]
 * @return ESP_OK, ESP_ERR_TIMEOUT nếu queue đầy, ESP_ERR_INVALID_ARG nếu len>8
 */
esp_err_t can_send_command(uint32_t can_id, const uint8_t *data, uint8_t len);

/**
 * @brief FreeRTOS task — Dequeue frame từ xCANTxQueue và truyền lên CAN bus.
 *        Block vô thời hạn trên queue.
 *
 * @note  Stack: 4096 bytes, Priority: 4
 * @param pvParam  Không dùng, truyền NULL
 */
void can_tx_task(void *pvParam);

/**
 * @brief FreeRTOS task — Block chờ frame từ CAN bus (twai_receive).
 *        Frame emergency → set xEmergencyGroup ngay (không queue).
 *        Frame thường     → push vào xCANRxQueue.
 *
 * @note  Stack: 4096 bytes, Priority: 5 (cao hơn CANTxTask)
 * @param pvParam  Không dùng, truyền NULL
 */
void can_rx_task(void *pvParam);

#ifdef __cplusplus
}
#endif
