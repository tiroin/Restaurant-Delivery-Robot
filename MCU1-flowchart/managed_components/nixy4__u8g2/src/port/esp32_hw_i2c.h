#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "u8g2.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef U8G2_ESP32_PIN_UNUSED
#define U8G2_ESP32_PIN_UNUSED (-1)
#endif

typedef struct {
	int i2c_port;
	int sda_pin;
	int scl_pin;
	uint32_t clk_hz;
	uint8_t dev_addr_7bit;
	int timeout_ms;
	int reset_pin;
} u8g2_esp32_i2c_config_t;

typedef struct {
	u8g2_esp32_i2c_config_t cfg;
	void *bus_handle;
	void *dev_handle;
	uint8_t tx_buf[256];
	uint16_t tx_len;
	uint8_t current_addr_7bit;
	uint8_t initialized;
} u8g2_esp32_i2c_ctx_t;

#define U8G2_ESP32_I2C_CONFIG_DEFAULT() \
	{                                   \
		.i2c_port = 0,                  \
		.sda_pin = 21,                  \
		.scl_pin = 22,                  \
		.clk_hz = 400000,               \
		.dev_addr_7bit = 0x3C,          \
		.timeout_ms = 1000,             \
		.reset_pin = U8G2_ESP32_PIN_UNUSED, \
	}

esp_err_t u8g2_esp32_i2c_set_default_context(u8g2_esp32_i2c_ctx_t *ctx);

uint8_t u8x8_byte_esp32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_esp32_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#ifdef __cplusplus
}
#endif
