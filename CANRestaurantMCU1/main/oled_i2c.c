#include "oled_i2c.h"
#include <string.h>

static const char *TAG = "OLED_I2C";
static i2c_port_t oled_i2c_port = I2C_NUM_0;
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// Font 5x7 cơ bản (ASCII 32-127)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // "\"
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
};

// Khởi tạo I2C
esp_err_t oled_init(oled_config_t *config) {
    oled_i2c_port = config->i2c_port;
    
    // Không khởi tạo lại I2C driver vì keypad đã khởi tạo rồi
    // Chỉ cần lưu port để sử dụng
    
    // Khởi tạo OLED
    vTaskDelay(pdMS_TO_TICKS(100));
    
    oled_write_command(OLED_CMD_DISPLAY_OFF);
    oled_write_command(OLED_CMD_SET_DISPLAY_CLOCK_DIV);
    oled_write_command(0x80);
    oled_write_command(OLED_CMD_SET_MULTIPLEX);
    oled_write_command(OLED_HEIGHT - 1);
    oled_write_command(OLED_CMD_SET_DISPLAY_OFFSET);
    oled_write_command(0x00);
    oled_write_command(OLED_CMD_SET_START_LINE | 0x00);
    oled_write_command(OLED_CMD_CHARGE_PUMP);
    oled_write_command(0x14);
    oled_write_command(OLED_CMD_MEMORY_MODE);
    oled_write_command(0x00);
    oled_write_command(OLED_CMD_SEG_REMAP | 0x01);
    oled_write_command(OLED_CMD_COM_SCAN_DEC);
    oled_write_command(OLED_CMD_SET_COM_PINS);
    oled_write_command(0x12);
    oled_write_command(OLED_CMD_SET_CONTRAST);
    oled_write_command(0xCF);
    oled_write_command(OLED_CMD_SET_PRECHARGE);
    oled_write_command(0xF1);
    oled_write_command(OLED_CMD_SET_VCOM_DETECT);
    oled_write_command(0x40);
    oled_write_command(OLED_CMD_DISPLAY_ALL_ON_RESUME);
    oled_write_command(OLED_CMD_NORMAL_DISPLAY);
    oled_write_command(OLED_CMD_DEACTIVATE_SCROLL);
    oled_write_command(OLED_CMD_DISPLAY_ON);
    
    oled_clear_screen();
    
    ESP_LOGI(TAG, "OLED initialized successfully");
    return ESP_OK;
}

// Gửi lệnh đến OLED
esp_err_t oled_write_command(uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Control byte cho command
    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(oled_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Gửi dữ liệu đến OLED
esp_err_t oled_write_data(uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); // Control byte cho data
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(oled_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Xóa màn hình
esp_err_t oled_clear_screen(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
    return oled_update_screen();
}

// Cập nhật toàn bộ màn hình từ buffer
esp_err_t oled_update_screen(void) {
    oled_write_command(OLED_CMD_COLUMN_ADDR);
    oled_write_command(0);
    oled_write_command(OLED_WIDTH - 1);
    oled_write_command(OLED_CMD_PAGE_ADDR);
    oled_write_command(0);
    oled_write_command((OLED_HEIGHT / 8) - 1);
    
    for (uint16_t i = 0; i < sizeof(oled_buffer); i += 16) {
        oled_write_data(&oled_buffer[i], 16);
    }
    return ESP_OK;
}

// Vẽ pixel
esp_err_t oled_draw_pixel(uint8_t x, uint8_t y, bool on) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (on) {
        oled_buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
    } else {
        oled_buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
    
    return ESP_OK;
}

// Vẽ đường thẳng (Bresenham's line algorithm)
esp_err_t oled_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool on) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (true) {
        oled_draw_pixel(x0, y0, on);
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return ESP_OK;
}

// Vẽ hình chữ nhật
esp_err_t oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on) {
    oled_draw_line(x, y, x + w - 1, y, on);
    oled_draw_line(x + w - 1, y, x + w - 1, y + h - 1, on);
    oled_draw_line(x + w - 1, y + h - 1, x, y + h - 1, on);
    oled_draw_line(x, y + h - 1, x, y, on);
    return ESP_OK;
}

// Vẽ hình chữ nhật đặc
esp_err_t oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on) {
    for (uint8_t i = 0; i < h; i++) {
        oled_draw_line(x, y + i, x + w - 1, y + i, on);
    }
    return ESP_OK;
}

// Vẽ hình tròn (Midpoint circle algorithm)
esp_err_t oled_draw_circle(uint8_t x0, uint8_t y0, uint8_t r, bool on) {
    int16_t x = r;
    int16_t y = 0;
    int16_t err = 0;
    
    while (x >= y) {
        oled_draw_pixel(x0 + x, y0 + y, on);
        oled_draw_pixel(x0 + y, y0 + x, on);
        oled_draw_pixel(x0 - y, y0 + x, on);
        oled_draw_pixel(x0 - x, y0 + y, on);
        oled_draw_pixel(x0 - x, y0 - y, on);
        oled_draw_pixel(x0 - y, y0 - x, on);
        oled_draw_pixel(x0 + y, y0 - x, on);
        oled_draw_pixel(x0 + x, y0 - y, on);
        
        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
    
    return ESP_OK;
}

// Viết ký tự
esp_err_t oled_write_char(char c, uint8_t x, uint8_t y) {
    if (c < 32 || c > 127) c = 32; // Chỉ hỗ trợ ASCII 32-127
    
    const uint8_t *char_data = font5x7[c - 32];
    
    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            if (char_data[i] & (1 << j)) {
                oled_draw_pixel(x + i, y + j, true);
            }
        }
    }
    
    return ESP_OK;
}

// Viết chuỗi
esp_err_t oled_write_string(const char *str, uint8_t x, uint8_t y) {
    uint8_t cursor_x = x;
    
    while (*str) {
        if (cursor_x > OLED_WIDTH - 6) break;
        oled_write_char(*str, cursor_x, y);
        cursor_x += 6;
        str++;
    }
    
    return ESP_OK;
}

// Bật/tắt màn hình
esp_err_t oled_display_on(bool on) {
    return oled_write_command(on ? OLED_CMD_DISPLAY_ON : OLED_CMD_DISPLAY_OFF);
}

// Đảo màu hiển thị
esp_err_t oled_invert_display(bool invert) {
    return oled_write_command(invert ? OLED_CMD_INVERT_DISPLAY : OLED_CMD_NORMAL_DISPLAY);
}

// Điều chỉnh độ tương phản
esp_err_t oled_set_contrast(uint8_t contrast) {
    oled_write_command(OLED_CMD_SET_CONTRAST);
    return oled_write_command(contrast);
}