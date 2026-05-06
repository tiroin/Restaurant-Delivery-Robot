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
	int host;
	int sclk_pin;
	int mosi_pin;
	int miso_pin;
	int cs_pin;
	int dc_pin;
	int reset_pin;
	int spi_mode;
	uint32_t clock_hz;
	int queue_size;
} u8g2_esp32_spi_config_t;

typedef struct {
	u8g2_esp32_spi_config_t cfg;
	void *dev_handle;
	uint8_t dc_level;
	uint8_t initialized;
} u8g2_esp32_spi_ctx_t;

#define U8G2_ESP32_SPI_CONFIG_DEFAULT() \
	{                                   \
		.host = 2,                      \
		.sclk_pin = 14,                 \
		.mosi_pin = 13,                 \
		.miso_pin = U8G2_ESP32_PIN_UNUSED, \
		.cs_pin = 15,                   \
		.dc_pin = U8G2_ESP32_PIN_UNUSED, \
		.reset_pin = U8G2_ESP32_PIN_UNUSED, \
		.spi_mode = 0,                  \
		.clock_hz = 10000000,           \
		.queue_size = 1,                \
	}

esp_err_t u8g2_esp32_spi_set_default_context(u8g2_esp32_spi_ctx_t *ctx);

uint8_t u8x8_byte_esp32_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_esp32_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#ifdef __cplusplus
}
#endif
