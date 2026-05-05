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

/* ════════════════════════════════════════════════════════════════
 *  TEACH-AND-REPLAY — Manual Learning Protocol
 * ════════════════════════════════════════════════════════════════ */

/* ── MCU1 (ESP32) → MCU2 (Motion MCU) ── */

/** Manual move command.
 *  data[0]   = dir  (0=FWD, 1=BWD, 2=LEFT, 3=RIGHT)
 *  data[1:2] = steps uint16 big-endian  (0 = stop)
 *  data[3:4] = speed uint16 big-endian  (TIM period µs; smaller = faster)
 */
#define CAN_ID_MANUAL_MOVE      0x110U

/** Save current position as checkpoint.
 *  data[0] = checkpoint_id (0=Home, 1..15=table slots)
 */
#define CAN_ID_SAVE_CP          0x111U

/** Clear a saved checkpoint from flash.
 *  data[0] = checkpoint_id
 */
#define CAN_ID_CLEAR_CP         0x112U

/** Set MCU2 operating mode.
 *  data[0] = 0(IDLE) | 1(LEARN) | 2(AUTO)
 */
#define CAN_ID_SET_MODE         0x113U

/* ── MCU2 (Motion MCU) → MCU1 (ESP32) ── */

/** Periodic telemetry broadcast (every 200 ms in LEARN/AUTO mode).
 *  data[0:1] = steps_L  int16 big-endian  (signed step count left motor)
 *  data[2:3] = steps_R  int16 big-endian  (signed step count right motor)
 *  data[4:5] = heading×10  int16 big-endian  (degrees × 10)
 *  data[6]   = mode   (0/1/2)
 *  data[7]   = last_cp id
 */
#define CAN_ID_TELEMETRY        0x210U

/** IMU accelerometer frame (every 200 ms).
 *  data[0:1] = accel_x × 100  int16 big-endian  (0.01 g)
 *  data[2:3] = accel_y × 100  int16 big-endian  (0.01 g)
 *  data[4:5] = accel_z × 100  int16 big-endian  (0.01 g)
 */
#define CAN_ID_IMU_ACCEL        0x211U

/** IMU gyroscope frame (every 200 ms).
 *  data[0:1] = gyro_x × 100   int16 big-endian  (0.01 dps)
 *  data[2:3] = gyro_y × 100   int16 big-endian  (0.01 dps)
 *  data[4:5] = gyro_z × 100   int16 big-endian  (0.01 dps)
 */
#define CAN_ID_IMU_GYRO         0x213U

/** Acknowledgment after a SAVE_CP command.
 *  data[0] = checkpoint_id
 *  data[1] = result  (0 = OK, 1 = FAIL)
 */
#define CAN_ID_CP_SAVED_ACK     0x212U

/** Odometry frame — sent by MCU2 when a motor segment ends.
 *  Sent on: explicit stop command, step-count target reached, or 300 ms timeout.
 *  data[0]   = last direction (0=FWD 1=BWD 2=LEFT 3=RIGHT)
 *  data[1:2] = motor1_steps  uint16 big-endian  (full steps since last start)
 *  data[3:4] = motor2_steps  uint16 big-endian
 *  data[5:6] = HAL_GetTick() & 0xFFFF  uint16 big-endian  (ms timestamp, wraps)
 */
#define CAN_ID_ODOMETRY         0x214U

/* MCU2 operating modes (used in CAN_ID_SET_MODE and CAN_ID_TELEMETRY) */
#define MCU2_MODE_IDLE          0x00U
#define MCU2_MODE_LEARN         0x01U
#define MCU2_MODE_AUTO          0x02U

/* Manual move direction codes (used in CAN_ID_MANUAL_MOVE data[0]) */
#define MOVE_DIR_FWD            0x00U
#define MOVE_DIR_BWD            0x01U
#define MOVE_DIR_LEFT           0x02U
#define MOVE_DIR_RIGHT          0x03U

#ifdef __cplusplus
}
#endif
