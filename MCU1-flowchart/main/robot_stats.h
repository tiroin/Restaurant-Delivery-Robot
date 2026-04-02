/**
 * @file    robot_stats.h
 * @brief   Robot runtime statistics — Public API
 *
 * Module nhỏ quản lý struct robot_stats_t global.
 * Tất cả task đọc/ghi stats thông qua các hàm này,
 * thread-safe bằng mutex nội bộ.
 *
 * Sử dụng:
 * @code
 *   robot_stats_init();               // Gọi một lần trong app_main
 *
 *   robot_stats_set_state(ROBOT_STATE_GOING_TABLE);
 *   robot_stats_inc_orders();
 *
 *   robot_stats_t snap;
 *   robot_stats_snapshot(&snap);      // Lấy bản sao an toàn để đọc
 *   ESP_LOGI(TAG, "State: %s", robot_state_to_str(snap.current_state));
 * @endcode
 */

#pragma once

#include "robot_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo mutex nội bộ và reset stats về giá trị mặc định.
 * @note  Phải gọi TRƯỚC tất cả hàm khác trong module này.
 */
void robot_stats_init(void);

/**
 * @brief Lấy bản sao (snapshot) an toàn của toàn bộ stats.
 *        Không block lâu — chỉ copy rồi unlock mutex.
 * @param[out] out  Buffer nhận snapshot
 */
void robot_stats_snapshot(robot_stats_t *out);

/* ── Setters (thread-safe) ────────────────────────────────── */

/** Cập nhật trạng thái robot */
void robot_stats_set_state(robot_state_t state);

/** Cập nhật checkpoint hiện tại */
void robot_stats_set_checkpoint(uint8_t checkpoint_id);

/** Cập nhật phần trăm pin */
void robot_stats_set_battery(uint8_t percent);

/** Ghi chuỗi lỗi cuối (vsnprintf-safe, cắt nếu dài hơn 63 ký tự) */
void robot_stats_set_error(const char *fmt, ...);

/* ── Incrementers (thread-safe) ───────────────────────────── */
void robot_stats_inc_orders(void);
void robot_stats_inc_trips(void);
void robot_stats_inc_emergency(void);
void robot_stats_inc_obstacle(void);

/* ── Helpers ──────────────────────────────────────────────── */

/**
 * @brief Chuyển robot_state_t thành chuỗi ASCII.
 * @return Con trỏ tới string literal, không cần free.
 */
const char *robot_state_to_str(robot_state_t state);

#ifdef __cplusplus
}
#endif
