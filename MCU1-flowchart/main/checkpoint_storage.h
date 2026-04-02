/**
 * @file    checkpoint_storage.h
 * @brief   Lưu trữ checkpoint vào NVS Flash — Public API
 *
 * Sử dụng:
 * @code
 *   checkpoint_storage_init();          // Gọi một lần trong app_main
 *
 *   checkpoint_t cp = {
 *       .id = 5, .name = "Ban 5",
 *       .pos_x = 3500.0f, .pos_y = 1200.0f, .valid = true
 *   };
 *   checkpoint_save(&cp);
 *
 *   checkpoint_t loaded;
 *   checkpoint_load(5, &loaded);
 *
 *   checkpoint_print_all();             // In ra Serial monitor
 * @endcode
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "robot_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo NVS flash và tạo checkpoint Home/Kitchen mặc định
 *        nếu chưa tồn tại.
 * @note  Phải gọi TRƯỚC tất cả các hàm khác trong module này.
 * @return ESP_OK nếu thành công
 */
esp_err_t checkpoint_storage_init(void);

/**
 * @brief Lưu (hoặc ghi đè) một checkpoint vào NVS flash.
 * @param[in]  cp  Con trỏ tới checkpoint cần lưu. cp->id phải < CHECKPOINT_MAX.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, hoặc lỗi NVS
 */
esp_err_t checkpoint_save(const checkpoint_t *cp);

/**
 * @brief Đọc một checkpoint từ NVS flash theo ID.
 * @param[in]  id  ID checkpoint cần đọc [0..CHECKPOINT_MAX-1]
 * @param[out] cp  Buffer nhận dữ liệu checkpoint
 * @return ESP_OK nếu tìm thấy, ESP_ERR_NVS_NOT_FOUND nếu chưa lưu
 */
esp_err_t checkpoint_load(uint8_t id, checkpoint_t *cp);

/**
 * @brief Đọc tất cả checkpoint hợp lệ từ flash.
 * @param[out] out    Mảng buffer nhận checkpoint, phải có ít nhất CHECKPOINT_MAX phần tử
 * @param[out] count  Số checkpoint thực tế đọc được
 * @return ESP_OK
 */
esp_err_t checkpoint_load_all(checkpoint_t *out, uint8_t *count);

/**
 * @brief Xóa một checkpoint khỏi NVS flash.
 * @note  Không thể xóa CHECKPOINT_ID_HOME (0) và CHECKPOINT_ID_KITCHEN (1).
 * @param[in]  id  ID checkpoint cần xóa
 * @return ESP_OK, ESP_ERR_NOT_ALLOWED nếu cố xóa Home/Kitchen
 */
esp_err_t checkpoint_delete(uint8_t id);

/**
 * @brief In danh sách tất cả checkpoint ra ESP_LOG (INFO level).
 */
void checkpoint_print_all(void);

#ifdef __cplusplus
}
#endif
