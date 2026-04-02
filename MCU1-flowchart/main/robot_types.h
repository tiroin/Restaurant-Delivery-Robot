/**
 * @file    robot_types.h
 * @brief   Shared types, enums, structs — Robot Restaurant Controller
 *
 * File này chỉ chứa định nghĩa kiểu dữ liệu dùng chung.
 * Không có bất kỳ extern, handle, hay logic nào ở đây.
 * Include file này ở BẤT KỲ module nào cần dùng kiểu dữ liệu.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ════════════════════════════════════════════════════════════════
 *  CHECKPOINT
 * ════════════════════════════════════════════════════════════════ */

#define CHECKPOINT_MAX          16      /**< Số checkpoint tối đa lưu flash */
#define CHECKPOINT_NAME_LEN     24      /**< Độ dài tên checkpoint (bao gồm '\0') */

#define CHECKPOINT_ID_HOME      0       /**< ID cố định: vị trí Home / sạc */
#define CHECKPOINT_ID_KITCHEN   1       /**< ID cố định: vị trí Bếp */

/**
 * @brief Một checkpoint vật lý trong không gian làm việc của robot.
 *        Tọa độ pos_x/pos_y tính bằng mm, dùng để đồng bộ sang Motion MCU.
 */
typedef struct {
    uint8_t  id;                         /**< ID duy nhất [0..CHECKPOINT_MAX-1] */
    char     name[CHECKPOINT_NAME_LEN];  /**< Tên hiển thị, vd "Ban 3", "Bep" */
    float    pos_x;                      /**< Tọa độ X (mm) */
    float    pos_y;                      /**< Tọa độ Y (mm) */
    bool     valid;                      /**< true = đã được lưu hợp lệ */
} checkpoint_t;

/* ════════════════════════════════════════════════════════════════
 *  ORDER
 * ════════════════════════════════════════════════════════════════ */

#define ORDER_ITEMS_MAX         8       /**< Số món tối đa trong một đơn */
#define ORDER_ITEM_NAME_LEN     32      /**< Độ dài tên món (bao gồm '\0') */

/**
 * @brief Đơn hàng được gửi từ khách qua RainMaker QR app.
 */
typedef struct {
    uint32_t order_id;                                    /**< ID tự tăng */
    uint8_t  table_checkpoint_id;                         /**< Checkpoint bàn cần giao */
    char     items[ORDER_ITEMS_MAX][ORDER_ITEM_NAME_LEN]; /**< Danh sách món */
    uint8_t  item_count;                                  /**< Số món thực tế */
    uint32_t timestamp_s;                                 /**< Unix timestamp (s) */
    bool     urgent;                                      /**< true = ưu tiên */
} order_t;

/* ════════════════════════════════════════════════════════════════
 *  ROBOT STATE
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief Trạng thái hoạt động của robot.
 *        StateMachineTask chuyển trạng thái theo luồng này:
 *
 *   IDLE → GOING_KITCHEN → WAITING_LOAD → GOING_TABLE
 *        → WAITING_UNLOAD → RETURNING_HOME → IDLE
 *
 *   Bất kỳ state nào → EMERGENCY (emergency bit set)
 *   Bất kỳ state nào → ERROR     (timeout / CAN error)
 */
typedef enum {
    ROBOT_STATE_IDLE           = 0, /**< Chờ tại Home */
    ROBOT_STATE_GOING_KITCHEN,      /**< Di chuyển đến bếp */
    ROBOT_STATE_WAITING_LOAD,       /**< Đợi bếp đặt món */
    ROBOT_STATE_GOING_TABLE,        /**< Di chuyển đến bàn */
    ROBOT_STATE_WAITING_UNLOAD,     /**< Đợi khách lấy món */
    ROBOT_STATE_RETURNING_HOME,     /**< Đang về Home */
    ROBOT_STATE_EMERGENCY,          /**< Dừng khẩn cấp */
    ROBOT_STATE_ERROR,              /**< Lỗi hệ thống */
    ROBOT_STATE_CHARGING,           /**< Đang sạc pin */
    ROBOT_STATE_MAX,
} robot_state_t;

/* ════════════════════════════════════════════════════════════════
 *  ROBOT STATS
 * ════════════════════════════════════════════════════════════════ */

/**
 * @brief Thống kê runtime của robot. Cập nhật bởi StateMachineTask,
 *        đọc bởi StatusBroadcastTask.
 */
typedef struct {
    uint32_t     total_orders_served;  /**< Tổng đơn đã phục vụ */
    uint32_t     total_trips;          /**< Tổng số chuyến đi */
    uint32_t     emergency_stops;      /**< Số lần dừng khẩn cấp */
    uint32_t     obstacle_detections;  /**< Số lần phát hiện vật cản */
    uint8_t      battery_percent;      /**< Pin hiện tại (%) */
    uint8_t      current_checkpoint;   /**< Checkpoint robot đang ở */
    robot_state_t current_state;       /**< State hiện tại */
    char         last_error[64];       /**< Mô tả lỗi cuối cùng */
} robot_stats_t;

/* ════════════════════════════════════════════════════════════════
 *  CAN BUS PROTOCOL  (11-bit standard ID)
 * ════════════════════════════════════════════════════════════════ */

/* ── ESP32 → Motion MCU ── */
#define CAN_ID_MOVE_TO_CP       0x100U  /**< data[0]=checkpoint_id */
#define CAN_ID_STOP_NOW         0x101U  /**< Emergency stop, no data */
#define CAN_ID_RETURN_HOME      0x102U  /**< No data */
#define CAN_ID_SET_SPEED        0x103U  /**< data[0]=speed 0-100% */
#define CAN_ID_REQ_STATUS       0x104U  /**< No data, request status reply */
#define CAN_ID_SET_CP_POS       0x105U  /**< data[0]=id, data[1-4]=x(float), data[5-8]=y(float) */

/* ── Motion MCU → ESP32 ── */
#define CAN_ID_ARRIVED_CP       0x200U  /**< data[0]=checkpoint_id arrived */
#define CAN_ID_MOTION_STATUS    0x201U  /**< data[0]=status_code, data[1]=battery% */
#define CAN_ID_OBSTACLE         0x202U  /**< Obstacle detected, no extra data */
#define CAN_ID_EMERGENCY        0x203U  /**< MCU self-triggered emergency stop */
#define CAN_ID_MOTION_ERROR     0x204U  /**< data[0]=error_code */

/* Motion status codes (data[0] của CAN_ID_MOTION_STATUS) */
#define MOTION_STATUS_IDLE      0x00U
#define MOTION_STATUS_MOVING    0x01U
#define MOTION_STATUS_ARRIVED   0x02U
#define MOTION_STATUS_OBSTACLE  0x03U
#define MOTION_STATUS_ERROR     0x04U

#ifdef __cplusplus
}
#endif
