/**
 * @file    app_main.c
 * @brief   MCU1 — Simple CAN + OLED test (3 tasks)
 *
 *  canRxTask  (prio 3): blocks on twai_receive → pushes to oledQueue
 *  canTxTask  (prio 2): blocks on vTaskDelay 2s → sends heartbeat 0x105
 *  oledTask   (prio 1): blocks on oledQueue 1s timeout → draws on OLED
 *
 *  TWAI:  TX=GPIO5   RX=GPIO4   @1Mbps  (matches MCU2/MCU3)
 *  OLED:  SDA=GPIO21 SCL=GPIO22  SSD1306 128x64 I2C
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/twai.h"

#include <u8g2.h>
#include "u8g2_esp32_hal.h"

/* ── Pin definitions ───────────────────────── */
#define PIN_SDA     21
#define PIN_SCL     22
#define PIN_CAN_TX  5
#define PIN_CAN_RX  4

/* ── CAN IDs (matching MCU2/MCU3) ──────────── */
#define CAN_ID_HEARTBEAT_MCU1   0x105

static const char *TAG = "MCU1";

/* ── Message passed from canRxTask → oledTask ─ */
typedef struct {
    uint32_t can_id;
    uint8_t  data[8];
    uint8_t  len;
} oled_msg_t;

/* ── Shared handles ────────────────────────── */
static QueueHandle_t oledQueue = NULL;
static u8g2_t u8g2;

/* ══════════════════════════════════════════════
 *  canRxTask — Priority 3
 *  BLOCKS ON: twai_receive() (driver handles ISR internally)
 *  UNBLOCKED BY: CAN frame arriving on bus
 * ══════════════════════════════════════════════ */
static void canRxTask(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "[canRx] Task started");
    twai_message_t rx;

    for (;;) {
        esp_err_t err = twai_receive(&rx, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "[canRx] err: %s", esp_err_to_name(err));
            continue;
        }

        ESP_LOGI(TAG, "[canRx] ID=0x%03lX len=%d data[0]=0x%02X",
                 (unsigned long)rx.identifier,
                 rx.data_length_code,
                 rx.data_length_code > 0 ? rx.data[0] : 0);

        oled_msg_t msg = {
            .can_id = rx.identifier,
            .len    = rx.data_length_code,
        };
        memcpy(msg.data, rx.data, 8);

        /* Overwrite — OLED always shows the latest frame */
        xQueueOverwrite(oledQueue, &msg);
    }
}

/* ══════════════════════════════════════════════
 *  canTxTask — Priority 2
 *  BLOCKS ON: vTaskDelay(2000ms)
 *  Sends heartbeat 0x105 with incrementing counter
 * ══════════════════════════════════════════════ */
static void canTxTask(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "[canTx] Task started");
    uint8_t counter = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        twai_message_t frame = {
            .identifier       = CAN_ID_HEARTBEAT_MCU1,
            .data_length_code = 1,
            .data             = { counter++ },
        };

        esp_err_t err = twai_transmit(&frame, pdMS_TO_TICKS(100));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "[canTx] Heartbeat #%d sent", counter - 1);
        } else {
            ESP_LOGW(TAG, "[canTx] TX fail: %s", esp_err_to_name(err));
        }
    }
}

/* ══════════════════════════════════════════════
 *  oledTask — Priority 1
 *  BLOCKS ON: xQueueReceive(oledQueue, 1000ms timeout)
 *  UNBLOCKED BY: canRxTask overwriting queue, or timeout
 * ══════════════════════════════════════════════ */
static void oledTask(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "[oled] Task started");

    oled_msg_t msg;
    uint32_t rx_count = 0;

    for (;;) {
        BaseType_t got = xQueueReceive(oledQueue, &msg, pdMS_TO_TICKS(1000));

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

        /* Header */
        u8g2_DrawStr(&u8g2, 0, 10, "MCU1 CAN Monitor");
        u8g2_DrawHLine(&u8g2, 0, 12, 128);

        if (got == pdTRUE) {
            rx_count++;
            char line[32];

            snprintf(line, sizeof(line), "ID: 0x%03lX  len:%d",
                     (unsigned long)msg.can_id, msg.len);
            u8g2_DrawStr(&u8g2, 0, 26, line);

            snprintf(line, sizeof(line), "D: %02X %02X %02X %02X",
                     msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
            u8g2_DrawStr(&u8g2, 0, 38, line);

            snprintf(line, sizeof(line), "   %02X %02X %02X %02X",
                     msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
            u8g2_DrawStr(&u8g2, 0, 50, line);

            snprintf(line, sizeof(line), "RX count: %lu",
                     (unsigned long)rx_count);
            u8g2_DrawStr(&u8g2, 0, 62, line);
        } else {
            u8g2_DrawStr(&u8g2, 0, 30, "Waiting for CAN...");
            char line[32];
            snprintf(line, sizeof(line), "RX total: %lu",
                     (unsigned long)rx_count);
            u8g2_DrawStr(&u8g2, 0, 50, line);
        }

        u8g2_SendBuffer(&u8g2);
    }
}

/* ══════════════════════════════════════════════
 *  canAlertTask — Priority 1
 *  Monitors TWAI bus errors / state changes
 * ══════════════════════════════════════════════ */
static void canAlertTask(void *arg)
{
    (void)arg;
    uint32_t alerts;
    twai_status_info_t status;

    /* Enable all alerts so we can see what's happening */
    twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);

    for (;;) {
        if (twai_read_alerts(&alerts, pdMS_TO_TICKS(5000)) == ESP_OK) {
            twai_get_status_info(&status);

            if (alerts & TWAI_ALERT_BUS_OFF)
                ESP_LOGE(TAG, "[alert] BUS OFF!");
            if (alerts & TWAI_ALERT_ERR_PASS)
                ESP_LOGW(TAG, "[alert] Error Passive");
            if (alerts & TWAI_ALERT_BUS_ERROR)
                ESP_LOGW(TAG, "[alert] Bus Error");
            if (alerts & TWAI_ALERT_TX_FAILED)
                ESP_LOGW(TAG, "[alert] TX Failed");
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
                ESP_LOGW(TAG, "[alert] RX Queue Full");
            if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
                ESP_LOGW(TAG, "[alert] Above Error Warning");
            if (alerts & TWAI_ALERT_ARB_LOST)
                ESP_LOGW(TAG, "[alert] Arbitration Lost");
            if (alerts & TWAI_ALERT_TX_SUCCESS)
                ESP_LOGD(TAG, "[alert] TX Success");
            if (alerts & TWAI_ALERT_RX_DATA)
                ESP_LOGD(TAG, "[alert] RX Data");

            ESP_LOGI(TAG, "[alert] state=%d tx_err=%lu rx_err=%lu tx_fail=%lu rx_miss=%lu",
                     status.state,
                     (unsigned long)status.tx_error_counter,
                     (unsigned long)status.rx_error_counter,
                     (unsigned long)status.tx_failed_count,
                     (unsigned long)status.rx_missed_count);
        }
    }
}

/* ── OLED init ─────────────────────────────── */
static void oled_init(void)
{
    u8g2_esp32_hal_t hal = U8G2_ESP32_HAL_DEFAULT;
    hal.bus.i2c.sda = PIN_SDA;
    hal.bus.i2c.scl = PIN_SCL;
    u8g2_esp32_hal_init(hal);

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2, U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    /* Splash */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 10, 30, "MCU1");
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(&u8g2, 10, 50, "CAN + OLED ready");
    u8g2_SendBuffer(&u8g2);

    ESP_LOGI(TAG, "OLED initialized");
}

/* ── TWAI init — 1Mbps matching MCU2/MCU3 ──── */
static esp_err_t can_init(void)
{
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX,
                                    (gpio_num_t)PIN_CAN_RX,
                                    TWAI_MODE_NORMAL);
    g_cfg.rx_queue_len = 16;
    g_cfg.tx_queue_len = 5;

    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "TWAI OK — TX:GPIO%d  RX:GPIO%d  @1Mbps",
             PIN_CAN_TX, PIN_CAN_RX);
    return ESP_OK;
}

/* ── app_main ──────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== MCU1 CAN + OLED Test ===");

    oled_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(can_init());

    oledQueue = xQueueCreate(1, sizeof(oled_msg_t));
    configASSERT(oledQueue);

    xTaskCreate(canRxTask,    "canRx",    4096, NULL, 3, NULL);
    xTaskCreate(canTxTask,    "canTx",    2048, NULL, 2, NULL);
    xTaskCreate(oledTask,     "oled",     4096, NULL, 1, NULL);
    xTaskCreate(canAlertTask, "canAlert", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "All tasks created");
}
