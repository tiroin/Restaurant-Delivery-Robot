#ifndef OLED_I2C_H
#define OLED_I2C_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_log.h"

// Địa chỉ I2C mặc định của OLED SSD1306
#define OLED_I2C_ADDRESS   0x3C

// Kích thước màn hình
#define OLED_WIDTH         128
#define OLED_HEIGHT        64

// SSD1306 Commands
#define OLED_CMD_SET_CONTRAST              0x81
#define OLED_CMD_DISPLAY_ALL_ON_RESUME     0xA4
#define OLED_CMD_DISPLAY_ALL_ON            0xA5
#define OLED_CMD_NORMAL_DISPLAY            0xA6
#define OLED_CMD_INVERT_DISPLAY            0xA7
#define OLED_CMD_DISPLAY_OFF               0xAE
#define OLED_CMD_DISPLAY_ON                0xAF
#define OLED_CMD_SET_DISPLAY_OFFSET        0xD3
#define OLED_CMD_SET_COM_PINS              0xDA
#define OLED_CMD_SET_VCOM_DETECT           0xDB
#define OLED_CMD_SET_DISPLAY_CLOCK_DIV     0xD5
#define OLED_CMD_SET_PRECHARGE             0xD9
#define OLED_CMD_SET_MULTIPLEX             0xA8
#define OLED_CMD_SET_LOW_COLUMN            0x00
#define OLED_CMD_SET_HIGH_COLUMN           0x10
#define OLED_CMD_SET_START_LINE            0x40
#define OLED_CMD_MEMORY_MODE               0x20
#define OLED_CMD_COLUMN_ADDR               0x21
#define OLED_CMD_PAGE_ADDR                 0x22
#define OLED_CMD_COM_SCAN_INC              0xC0
#define OLED_CMD_COM_SCAN_DEC              0xC8
#define OLED_CMD_SEG_REMAP                 0xA0
#define OLED_CMD_CHARGE_PUMP               0x8D
#define OLED_CMD_EXTERNAL_VCC              0x01
#define OLED_CMD_SWITCH_CAP_VCC            0x02
#define OLED_CMD_ACTIVATE_SCROLL           0x2F
#define OLED_CMD_DEACTIVATE_SCROLL         0x2E
#define OLED_CMD_SET_VERTICAL_SCROLL_AREA  0xA3
#define OLED_CMD_RIGHT_HORIZONTAL_SCROLL   0x26
#define OLED_CMD_LEFT_HORIZONTAL_SCROLL    0x27

// Cấu trúc cấu hình OLED
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t clk_speed;
} oled_config_t;

// Các hàm khởi tạo và điều khiển cơ bản
esp_err_t oled_init(oled_config_t *config);
esp_err_t oled_write_command(uint8_t command);
esp_err_t oled_write_data(uint8_t *data, size_t len);
esp_err_t oled_clear_screen(void);
esp_err_t oled_set_cursor(uint8_t x, uint8_t y);
esp_err_t oled_display_on(bool on);
esp_err_t oled_invert_display(bool invert);
esp_err_t oled_set_contrast(uint8_t contrast);

// Các hàm vẽ
esp_err_t oled_draw_pixel(uint8_t x, uint8_t y, bool on);
esp_err_t oled_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool on);
esp_err_t oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);
esp_err_t oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);
esp_err_t oled_draw_circle(uint8_t x0, uint8_t y0, uint8_t r, bool on);

// Các hàm hiển thị text
esp_err_t oled_write_char(char c, uint8_t x, uint8_t y);
esp_err_t oled_write_string(const char *str, uint8_t x, uint8_t y);

// Hàm cập nhật màn hình
esp_err_t oled_update_screen(void);

#endif // OLED_I2C_H