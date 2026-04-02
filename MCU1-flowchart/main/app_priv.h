/*
 * Robot Restaurant Controller — app_priv.h
 * ESP32 + ESP-IDF + FreeRTOS + ESP-RainMaker
 *
 * Kiến trúc 5 tasks:
 *   1. WiFiOrderTask      — Nhận lệnh QR/Order từ RainMaker MQTT
 *   2. StateMachineTask   — Quản lý trạng thái robot + checkpoints
 *   3. CANTxTask          — Gửi lệnh di chuyển đến Motion MCU qua CAN
 *   4. CANRxTask          — Nhận phản hồi từ Motion MCU qua CAN
 *   5. StatusBroadcastTask — Cập nhật trạng thái lên RainMaker + OLED
 *   6. WatchdogTask       — Feed ESP32 watchdog
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

/* ─── ESP-RainMaker device (extern, defined in app_main.c) ─── */
#include <esp_rmaker_core.h>
extern esp_rmaker_device_t *robot_device;
extern esp_rmaker_device_t *stats_device;

/* ─── CAN / TWAI driver ─── */
#include "driver/twai.h"

/* ════════════════════════════════════════════════════════
 *  ĐỊNH NGHĨA CÁC CHECKPOINT CỦA ROBOT
 * ════════════════════════════════════════════════════════ */
#define MAX_CHECKPOINTS     16      /* Số checkpoint tối đa lưu trong flash */
#define CHECKPOINT_NAME_LEN 24

typedef struct {
    uint8_t  id;                        /* ID duy nhất của checkpoint (0 = Home) */
    char     name[CHECKPOINT_NAME_LEN]; /* Tên hiển thị, vd "Bàn 3", "Bếp" */
    float    pos_x;                     /* Tọa độ X (mm), chỉ để tham khảo/log */
    float    pos_y;                     /* Tọa độ Y (mm) */
    bool     valid;                     /* true = checkpoint đã được lưu */
} Checkpoint;

/* Checkpoint đặc biệt */
#define CHECKPOINT_HOME_ID  0
#define CHECKPOINT_KITCHEN_ID  1

/* ════════════════════════════════════════════════════════
 *  ĐỊNH NGHĨA ĐƠN HÀNG (ORDER)
 * ════════════════════════════════════════════════════════ */
#define MAX_ORDER_ITEMS     8
#define ORDER_ITEM_NAME_LEN 32

typedef struct {
    uint32_t order_id;                          /* ID đơn hàng duy nhất */
    uint8_t  table_checkpoint_id;               /* Checkpoint bàn cần đến */
    char     items[MAX_ORDER_ITEMS][ORDER_ITEM_NAME_LEN]; /* Tên món */
    uint8_t  item_count;
    uint32_t timestamp;                         /* Thời điểm đặt hàng */
    bool     urgent;                            /* Ưu tiên cao */
} Order;

/* ════════════════════════════════════════════════════════
 *  TRẠNG THÁI ROBOT (State Machine)
 * ════════════════════════════════════════════════════════ */
typedef enum {
    ROBOT_STATE_IDLE        = 0,    /* Đang chờ tại Home */
    ROBOT_STATE_GOING_KITCHEN,      /* Di chuyển đến bếp lấy món */
    ROBOT_STATE_WAITING_LOAD,       /* Đợi bếp đặt món lên robot */
    ROBOT_STATE_GOING_TABLE,        /* Di chuyển đến bàn giao món */
    ROBOT_STATE_WAITING_UNLOAD,     /* Đợi khách lấy món */
    ROBOT_STATE_RETURNING_HOME,     /* Đang về Home */
    ROBOT_STATE_EMERGENCY,          /* Dừng khẩn cấp */
    ROBOT_STATE_ERROR,              /* Lỗi hệ thống */
    ROBOT_STATE_CHARGING,           /* Đang sạc pin */
} RobotState;

/* ════════════════════════════════════════════════════════
 *  CAN BUS — GIAO THỨC VỚI MOTION MCU
 *  (ID 11-bit standard frame)
 * ════════════════════════════════════════════════════════ */

/* CAN ID từ ESP32 → Motion MCU */
#define CAN_ID_MOVE_TO_CHECKPOINT   0x100   /* data[0] = checkpoint_id */
#define CAN_ID_STOP_IMMEDIATE       0x101   /* Dừng ngay (emergency) */
#define CAN_ID_RETURN_HOME          0x102   /* Về Home */
#define CAN_ID_SET_SPEED            0x103   /* data[0] = speed (0-100%) */
#define CAN_ID_REQUEST_STATUS       0x104   /* Yêu cầu báo cáo trạng thái */
#define CAN_ID_SET_CHECKPOINT_POS   0x105   /* Lưu vị trí checkpoint: data[0]=id, data[1-4]=x, data[5-8]=y */

/* CAN ID từ Motion MCU → ESP32 */
#define CAN_ID_ARRIVED_CHECKPOINT   0x200   /* data[0] = checkpoint_id đã đến */
#define CAN_ID_MOTION_STATUS        0x201   /* data[0] = status, data[1] = battery% */
#define CAN_ID_OBSTACLE_DETECTED    0x202   /* Phát hiện vật cản */
#define CAN_ID_EMERGENCY_STOP       0x203   /* MCU tự dừng khẩn cấp */
#define CAN_ID_MOTION_ERROR         0x204   /* data[0] = error code */

/* Motion MCU status byte */
#define MOTION_STATUS_IDLE          0x00
#define MOTION_STATUS_MOVING        0x01
#define MOTION_STATUS_ARRIVED       0x02
#define MOTION_STATUS_OBSTACLE      0x03
#define MOTION_STATUS_ERROR         0x04

/* ════════════════════════════════════════════════════════
 *  FreeRTOS PRIMITIVES (extern, defined in app_main.c)
 * ════════════════════════════════════════════════════════ */
extern QueueHandle_t     xOrderQueue;       /* WiFiOrderTask  → StateMachineTask */
extern QueueHandle_t     xCANTxQueue;       /* StateMachineTask → CANTxTask */
extern QueueHandle_t     xCANRxQueue;       /* CANRxTask → StateMachineTask */
extern EventGroupHandle_t xEmergencyGroup;  /* Bit 0: emergency stop */
extern SemaphoreHandle_t  xCANMailboxSem;   /* CAN Tx mailbox semaphore */

/* Emergency event group bits */
#define EMERGENCY_STOP_BIT  BIT0
#define OBSTACLE_BIT        BIT1

/* ════════════════════════════════════════════════════════
 *  THỐNG KÊ HỆ THỐNG
 * ════════════════════════════════════════════════════════ */
typedef struct {
    uint32_t total_orders_served;   /* Tổng số đơn hàng đã phục vụ */
    uint32_t total_trips;           /* Tổng số chuyến đi */
    uint32_t emergency_stops;       /* Số lần dừng khẩn cấp */
    uint32_t obstacle_detections;   /* Số lần phát hiện vật cản */
    uint8_t  battery_percent;       /* Pin hiện tại (%) */
    uint8_t  current_checkpoint;    /* Checkpoint hiện tại */
    RobotState current_state;       /* Trạng thái hiện tại */
    char     last_error[64];        /* Thông báo lỗi cuối */
} RobotStats;

extern RobotStats g_robot_stats;

/* ════════════════════════════════════════════════════════
 *  FUNCTION DECLARATIONS
 * ════════════════════════════════════════════════════════ */

/* app_main.c */
void app_rmaker_init(void);

/* can_tasks.c */
void can_driver_init(void);
void can_tx_task(void *pvParam);
void can_rx_task(void *pvParam);

/* robot_tasks.c */
void wifi_order_task(void *pvParam);
void state_machine_task(void *pvParam);
void status_broadcast_task(void *pvParam);
void watchdog_task(void *pvParam);

/* checkpoint_storage.c */
esp_err_t checkpoint_storage_init(void);
esp_err_t checkpoint_save(const Checkpoint *cp);
esp_err_t checkpoint_load(uint8_t id, Checkpoint *cp);
esp_err_t checkpoint_load_all(Checkpoint *out, uint8_t *count);
esp_err_t checkpoint_delete(uint8_t id);
void      checkpoint_print_all(void);

/* Helper */
const char *robot_state_str(RobotState s);
