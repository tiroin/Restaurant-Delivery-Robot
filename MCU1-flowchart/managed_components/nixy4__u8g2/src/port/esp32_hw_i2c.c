#include "esp32_hw_i2c.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "u8g2_i2c_port";

static u8g2_esp32_i2c_ctx_t *s_default_ctx;

static inline u8g2_esp32_i2c_ctx_t *get_ctx(void)
{
	return s_default_ctx;
}

static esp_err_t i2c_recreate_device_if_needed(u8g2_esp32_i2c_ctx_t *ctx, uint8_t addr_7bit)
{
	if (ctx->dev_handle != NULL && ctx->current_addr_7bit == addr_7bit) {
		return ESP_OK;
	}

	if (ctx->dev_handle != NULL) {
		i2c_master_bus_rm_device((i2c_master_dev_handle_t)ctx->dev_handle);
		ctx->dev_handle = NULL;
	}

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr_7bit,
		.scl_speed_hz = ctx->cfg.clk_hz,
		.scl_wait_us = 0,
		.flags.disable_ack_check = 0,
	};

	esp_err_t err = i2c_master_bus_add_device((i2c_master_bus_handle_t)ctx->bus_handle,
											   &dev_cfg,
											   (i2c_master_dev_handle_t *)&ctx->dev_handle);
	if (err == ESP_OK) {
		ctx->current_addr_7bit = addr_7bit;
	}
	return err;
}

static esp_err_t i2c_port_init(u8g2_esp32_i2c_ctx_t *ctx)
{
	if (ctx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	if (ctx->initialized) {
		return ESP_OK;
	}

	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = ctx->cfg.i2c_port,
		.sda_io_num = ctx->cfg.sda_pin,
		.scl_io_num = ctx->cfg.scl_pin,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.intr_priority = 0,
		.trans_queue_depth = 0,
		.flags.enable_internal_pullup = 1,
		.flags.allow_pd = 0,
	};

	esp_err_t err = i2c_new_master_bus(&bus_cfg, (i2c_master_bus_handle_t *)&ctx->bus_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
		return err;
	}

	err = i2c_recreate_device_if_needed(ctx, ctx->cfg.dev_addr_7bit);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
		i2c_del_master_bus((i2c_master_bus_handle_t)ctx->bus_handle);
		ctx->bus_handle = NULL;
		return err;
	}

	ctx->initialized = 1;
	ctx->tx_len = 0;
	return ESP_OK;
}

esp_err_t u8g2_esp32_i2c_set_default_context(u8g2_esp32_i2c_ctx_t *ctx)
{
	if (ctx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	s_default_ctx = ctx;
	return ESP_OK;
}

uint8_t u8x8_byte_esp32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	u8g2_esp32_i2c_ctx_t *ctx = get_ctx();
	if (ctx == NULL) {
		return 0;
	}

	switch (msg) {
	case U8X8_MSG_BYTE_INIT: {
		return i2c_port_init(ctx) == ESP_OK;
	}

	case U8X8_MSG_BYTE_SET_DC:
		return 1;

	case U8X8_MSG_BYTE_START_TRANSFER: {
		if (!ctx->initialized && i2c_port_init(ctx) != ESP_OK) {
			return 0;
		}

		uint8_t i2c_addr_8bit = u8x8_GetI2CAddress(u8x8);
		uint8_t i2c_addr_7bit = (uint8_t)(i2c_addr_8bit >> 1);
		esp_err_t err = i2c_recreate_device_if_needed(ctx, i2c_addr_7bit);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "I2C set address 0x%02X failed: %s", i2c_addr_7bit, esp_err_to_name(err));
			return 0;
		}

		ctx->tx_len = 0;
		return 1;
	}

	case U8X8_MSG_BYTE_SEND: {
		if (arg_ptr == NULL || arg_int == 0) {
			return 1;
		}

		const uint8_t *src = (const uint8_t *)arg_ptr;
		uint16_t remaining = arg_int;

		while (remaining > 0) {
			uint16_t room = (uint16_t)(sizeof(ctx->tx_buf) - ctx->tx_len);
			uint16_t chunk = remaining < room ? remaining : room;

			memcpy(&ctx->tx_buf[ctx->tx_len], src, chunk);
			ctx->tx_len += chunk;
			src += chunk;
			remaining -= chunk;

			if (ctx->tx_len == sizeof(ctx->tx_buf)) {
				esp_err_t err = i2c_master_transmit((i2c_master_dev_handle_t)ctx->dev_handle,
													ctx->tx_buf,
													ctx->tx_len,
													ctx->cfg.timeout_ms);
				if (err != ESP_OK) {
					ESP_LOGE(TAG, "i2c_master_transmit (partial) failed: %s", esp_err_to_name(err));
					ctx->tx_len = 0;
					return 0;
				}
				ctx->tx_len = 0;
			}
		}
		return 1;
	}

	case U8X8_MSG_BYTE_END_TRANSFER: {
		if (ctx->tx_len == 0) {
			return 1;
		}

		esp_err_t err = i2c_master_transmit((i2c_master_dev_handle_t)ctx->dev_handle,
											ctx->tx_buf,
											ctx->tx_len,
											ctx->cfg.timeout_ms);
		ctx->tx_len = 0;
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "i2c_master_transmit failed: %s", esp_err_to_name(err));
			return 0;
		}
		return 1;
	}

	default:
		return 1;
	}
}

uint8_t u8x8_gpio_and_delay_esp32_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)arg_ptr;
	(void)u8x8;
	u8g2_esp32_i2c_ctx_t *ctx = get_ctx();

	switch (msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		if (ctx != NULL && ctx->cfg.reset_pin >= 0) {
			gpio_config_t io_cfg = {
				.pin_bit_mask = (1ULL << ctx->cfg.reset_pin),
				.mode = GPIO_MODE_OUTPUT,
				.pull_up_en = GPIO_PULLUP_DISABLE,
				.pull_down_en = GPIO_PULLDOWN_DISABLE,
				.intr_type = GPIO_INTR_DISABLE,
			};
			gpio_config(&io_cfg);
		}
		return 1;

	case U8X8_MSG_GPIO_RESET:
		if (ctx != NULL && ctx->cfg.reset_pin >= 0) {
			gpio_set_level(ctx->cfg.reset_pin, arg_int);
		}
		return 1;

	case U8X8_MSG_DELAY_MILLI:
		vTaskDelay(pdMS_TO_TICKS(arg_int));
		return 1;

	case U8X8_MSG_DELAY_10MICRO:
		esp_rom_delay_us(10U * arg_int);
		return 1;

	case U8X8_MSG_DELAY_NANO:
	case U8X8_MSG_DELAY_100NANO:
		return 1;

	default:
		return 1;
	}
}
