#include "esp32_hw_spi.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "u8g2_spi_port";

static u8g2_esp32_spi_ctx_t *s_default_ctx;

static inline u8g2_esp32_spi_ctx_t *get_ctx(void)
{
	return s_default_ctx;
}

static esp_err_t spi_port_init(u8g2_esp32_spi_ctx_t *ctx)
{
	if (ctx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	if (ctx->initialized) {
		return ESP_OK;
	}

	spi_bus_config_t bus_cfg = {
		.mosi_io_num = ctx->cfg.mosi_pin,
		.miso_io_num = ctx->cfg.miso_pin,
		.sclk_io_num = ctx->cfg.sclk_pin,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.data4_io_num = -1,
		.data5_io_num = -1,
		.data6_io_num = -1,
		.data7_io_num = -1,
		.max_transfer_sz = 0,
		.flags = SPICOMMON_BUSFLAG_MASTER,
		.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
		.intr_flags = 0,
	};

	esp_err_t err = spi_bus_initialize((spi_host_device_t)ctx->cfg.host,
									   &bus_cfg,
									   SPI_DMA_CH_AUTO);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
		return err;
	}

	spi_device_interface_config_t dev_cfg = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = ctx->cfg.spi_mode,
		.clock_source = SPI_CLK_SRC_DEFAULT,
		.duty_cycle_pos = 128,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = (int)ctx->cfg.clock_hz,
		.input_delay_ns = 0,
		.spics_io_num = ctx->cfg.cs_pin,
		.flags = 0,
		.queue_size = ctx->cfg.queue_size,
		.pre_cb = NULL,
		.post_cb = NULL,
	};

	err = spi_bus_add_device((spi_host_device_t)ctx->cfg.host,
							 &dev_cfg,
							 (spi_device_handle_t *)&ctx->dev_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
		return err;
	}

	ctx->initialized = 1;
	ctx->dc_level = 0;
	return ESP_OK;
}

esp_err_t u8g2_esp32_spi_set_default_context(u8g2_esp32_spi_ctx_t *ctx)
{
	if (ctx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	s_default_ctx = ctx;
	return ESP_OK;
}

uint8_t u8x8_byte_esp32_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)u8x8;
	u8g2_esp32_spi_ctx_t *ctx = get_ctx();
	if (ctx == NULL) {
		return 0;
	}

	switch (msg) {
	case U8X8_MSG_BYTE_INIT:
		return spi_port_init(ctx) == ESP_OK;

	case U8X8_MSG_BYTE_SET_DC:
		ctx->dc_level = arg_int ? 1 : 0;
		if (ctx->cfg.dc_pin >= 0) {
			gpio_set_level(ctx->cfg.dc_pin, ctx->dc_level);
		}
		return 1;

	case U8X8_MSG_BYTE_START_TRANSFER:
		return 1;

	case U8X8_MSG_BYTE_SEND: {
		if (!ctx->initialized && spi_port_init(ctx) != ESP_OK) {
			return 0;
		}
		if (arg_ptr == NULL || arg_int == 0) {
			return 1;
		}

		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.length = arg_int * 8U;
		t.tx_buffer = arg_ptr;

		esp_err_t err = spi_device_polling_transmit((spi_device_handle_t)ctx->dev_handle, &t);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "spi_device_polling_transmit failed: %s", esp_err_to_name(err));
			return 0;
		}
		return 1;
	}

	case U8X8_MSG_BYTE_END_TRANSFER:
		return 1;

	default:
		return 1;
	}
}

uint8_t u8x8_gpio_and_delay_esp32_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)arg_ptr;
	(void)u8x8;
	u8g2_esp32_spi_ctx_t *ctx = get_ctx();

	switch (msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT: {
		uint64_t pin_mask = 0;
		if (ctx != NULL && ctx->cfg.dc_pin >= 0) {
			pin_mask |= (1ULL << ctx->cfg.dc_pin);
		}
		if (ctx != NULL && ctx->cfg.reset_pin >= 0) {
			pin_mask |= (1ULL << ctx->cfg.reset_pin);
		}
		if (pin_mask != 0) {
			gpio_config_t io_cfg = {
				.pin_bit_mask = pin_mask,
				.mode = GPIO_MODE_OUTPUT,
				.pull_up_en = GPIO_PULLUP_DISABLE,
				.pull_down_en = GPIO_PULLDOWN_DISABLE,
				.intr_type = GPIO_INTR_DISABLE,
			};
			gpio_config(&io_cfg);
		}
		return 1;
	}

	case U8X8_MSG_GPIO_DC:
		if (ctx != NULL && ctx->cfg.dc_pin >= 0) {
			ctx->dc_level = arg_int ? 1 : 0;
			gpio_set_level(ctx->cfg.dc_pin, ctx->dc_level);
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
