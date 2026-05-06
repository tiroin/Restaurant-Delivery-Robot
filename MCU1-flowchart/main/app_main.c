/**
 * @file    app_main.c
 * @brief   MCU1 — CAN monitor + OLED + WiFi AP joystick control
 *
 *  canRxTask  (prio 3): twai_receive → oledQueue + WS broadcast (0x210/211/212)
 *  canTxTask  (prio 2): sends heartbeat 0x105 every 2s
 *  oledTask   (prio 1): draws latest CAN frame on SSD1306
 *  canAlertTask (prio 1): logs TWAI bus errors
 *
 *  WiFi AP:   SSID "RobotControl"  Pass "robot1234"  IP 192.168.4.1
 *  WS server: ws://192.168.4.1/ws  (joystick + telemetry)
 *  TWAI:      TX=GPIO5  RX=GPIO4   @1Mbps
 *  OLED:      SDA=GPIO21 SCL=GPIO22  SSD1306 128×64 I2C
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/twai.h"

#include <u8g2.h>
#include "u8g2_esp32_hal.h"

#include "ble_robot.h"
#include "robot_types.h"
#include "wifi_ap_ws.h"

/* ── Pin definitions ───────────────────────── */
#define PIN_SDA     21
#define PIN_SCL     22
#define PIN_CAN_TX  5
#define PIN_CAN_RX  4

/* ── CAN IDs (matching MCU2/MCU3) ──────────── */
#define CAN_ID_HEARTBEAT_MCU1   0x105
#define CAN_ID_TEST_PING        0x100   /* MCU1 → MCU3 test ping */
#define CAN_ID_TEST_ECHO        0x200   /* MCU3 → MCU1 echo     */
/* CAN_ID_IMU_ACCEL 0x211 and CAN_ID_IMU_GYRO 0x213 come from robot_types.h */
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
static volatile uint8_t  g_tx_counter = 0;

/* ── Latest IMU values (updated by canRxTask) ─ */
static volatile int16_t g_ax = 0, g_ay = 0, g_az = 0;  /* *100, 0.01g */
static volatile int16_t g_gx = 0, g_gy = 0, g_gz = 0;  /* *10, 0.1dps */

/* ── Per-CAN-ID counters (updated by canRxTask) ─ */
static volatile uint32_t g_n_total    = 0;
static volatile uint32_t g_n_accel    = 0;  /* 0x211 */
static volatile uint32_t g_n_gyro     = 0;  /* 0x213 */
static volatile uint32_t g_n_telem    = 0;  /* 0x210 */
static volatile uint32_t g_n_ack      = 0;  /* 0x212 */
static volatile uint32_t g_n_echo     = 0;  /* 0x200 */
static volatile uint32_t g_n_other    = 0;
static volatile uint32_t g_last_other_id = 0;

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
        g_n_total++;

        ESP_LOGI(TAG, "[canRx] ID=0x%03lX len=%d data=%02X %02X %02X %02X %02X %02X %02X %02X",
                 (unsigned long)rx.identifier, rx.data_length_code,
                 rx.data[0], rx.data[1], rx.data[2], rx.data[3],
                 rx.data[4], rx.data[5], rx.data[6], rx.data[7]);

        /* ── Forward teach-and-replay telemetry to WebSocket clients ── */
        char ws_json[128];
        switch (rx.identifier) {

        case CAN_ID_TELEMETRY: {  /* 0x210 */
            g_n_telem++;
            int16_t steps_l = (int16_t)((rx.data[0] << 8) | rx.data[1]);
            int16_t steps_r = (int16_t)((rx.data[2] << 8) | rx.data[3]);
            int16_t heading = (int16_t)((rx.data[4] << 8) | rx.data[5]);
            snprintf(ws_json, sizeof(ws_json),
                     "{\"type\":\"telemetry\","
                     "\"steps_l\":%d,\"steps_r\":%d,"
                     "\"heading\":%d,\"mode\":%d,\"last_cp\":%d}",
                     steps_l, steps_r, heading, rx.data[6], rx.data[7]);
            ble_robot_notify(ws_json);
            break;
        }

        case CAN_ID_IMU_ACCEL: {  /* 0x211 — accelerometer */
            g_n_accel++;
            int16_t ax = (int16_t)((rx.data[0] << 8) | rx.data[1]);
            int16_t ay = (int16_t)((rx.data[2] << 8) | rx.data[3]);
            int16_t az = (int16_t)((rx.data[4] << 8) | rx.data[5]);
            g_ax = ax; g_ay = ay; g_az = az;
            ESP_LOGI(TAG, "[canRx] ACCEL ax=%.2fg ay=%.2fg az=%.2fg",
                     ax / 100.0f, ay / 100.0f, az / 100.0f);
            snprintf(ws_json, sizeof(ws_json),
                     "{\"type\":\"accel\",\"ax\":%d,\"ay\":%d,\"az\":%d}",
                     ax, ay, az);
            ble_robot_notify(ws_json);
            break;
        }

        case CAN_ID_IMU_GYRO: {  /* 0x213 — gyroscope: values in 0.1°/s units (divide by 10 for °/s) */
            g_n_gyro++;
            int16_t gx = (int16_t)((rx.data[0] << 8) | rx.data[1]);
            int16_t gy = (int16_t)((rx.data[2] << 8) | rx.data[3]);
            int16_t gz = (int16_t)((rx.data[4] << 8) | rx.data[5]);
            g_gx = gx; g_gy = gy; g_gz = gz;
            ESP_LOGI(TAG, "[canRx] GYRO  gx=%.1f gy=%.1f gz=%.1f deg/s",
                     gx / 10.0f, gy / 10.0f, gz / 10.0f);
            snprintf(ws_json, sizeof(ws_json),
                     "{\"type\":\"gyro\",\"gx\":%d,\"gy\":%d,\"gz\":%d}",
                     gx, gy, gz);
            ble_robot_notify(ws_json);
            break;
        }

        case CAN_ID_CP_SAVED_ACK:  /* 0x212 */
            g_n_ack++;
            ESP_LOGW(TAG, "[canRx] >>> CP_ACK cp_id=%u result=%u <<<",
                     rx.data[0], rx.data[1]);
            snprintf(ws_json, sizeof(ws_json),
                     "{\"type\":\"ack\",\"cp_id\":%d,\"result\":%d}",
                     rx.data[0], rx.data[1]);
            ble_robot_notify(ws_json);
            break;

        case CAN_ID_ODOMETRY: {  /* 0x214 — hardware step count on segment end */
            uint8_t  odo_dir = rx.data[0];
            uint16_t s1 = (uint16_t)((rx.data[1] << 8) | rx.data[2]);
            uint16_t s2 = (uint16_t)((rx.data[3] << 8) | rx.data[4]);
            ESP_LOGI(TAG, "[canRx] ODO  dir=%u s1=%u s2=%u", odo_dir, s1, s2);
            snprintf(ws_json, sizeof(ws_json),
                     "{\"type\":\"odo\",\"dir\":%d,\"s1\":%d,\"s2\":%d}",
                     odo_dir, s1, s2);
            ble_robot_notify(ws_json);
            break;
        }

        case CAN_ID_TEST_ECHO:  /* 0x200 */
            g_n_echo++;
            ESP_LOGI(TAG, "[canRx] echo from MCU3");
            break;

        case 0x301: {  /* OBSTACLE_ALERT from MCU3 (data[0]=0xA1) — front sensor <15cm */
            ESP_LOGW(TAG, "[canRx] >>> OBSTACLE 0x301 (front <15cm) <<<");
            wifi_ap_ws_set_emergency(true);  /* block move/stop from web UI */
            snprintf(ws_json, sizeof(ws_json),
                     "{\"type\":\"obstacle\",\"state\":1}");
            ble_robot_notify(ws_json);
            break;
        }

        case 0x303: {  /* PATH_CLEAR from MCU3 (data[0]=0xC1) — front clear */
            ESP_LOGI(TAG, "[canRx] >>> PATH_CLEAR 0x303 <<<");
            wifi_ap_ws_set_emergency(false);  /* re-enable move/stop from web UI */
            snprintf(ws_json, sizeof(ws_json),
                     "{\"type\":\"path_clear\",\"state\":0}");
            ble_robot_notify(ws_json);
            break;
        }

        default:
            g_n_other++;
            g_last_other_id = rx.identifier;
            break;
        }

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
 *  Sends test ping 0x100 to MCU3 every 1s
 *  data[0]=0xAA  data[1]=counter
 * ══════════════════════════════════════════════ */
static void canTxTask(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "[canTx] Task started — sending 0x100 to MCU3 every 1s");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        twai_message_t frame = {
            .identifier       = CAN_ID_TEST_PING,
            .data_length_code = 2,
            .data             = { 0xAA, g_tx_counter },
        };

        esp_err_t err = twai_transmit(&frame, pdMS_TO_TICKS(100));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "[canTx] PING 0x100 cnt=%d", g_tx_counter);
            g_tx_counter++;
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

        if (got == pdTRUE) rx_count++;

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

        /* Row 1: header */
        u8g2_DrawStr(&u8g2, 0, 10, "MCU1  BMI160 via CAN");
        u8g2_DrawHLine(&u8g2, 0, 12, 128);

        char line[32];

        /* Row 2: Accel */
        snprintf(line, sizeof(line), "A %+.2f %+.2f %+.2fg",
                 g_ax / 100.0f, g_ay / 100.0f, g_az / 100.0f);
        u8g2_DrawStr(&u8g2, 0, 24, line);

        /* Row 3: Gyro */
        snprintf(line, sizeof(line), "G %+.2f %+.2f %+.2fd",
                 g_gx / 100.0f, g_gy / 100.0f, g_gz / 100.0f);
        u8g2_DrawStr(&u8g2, 0, 36, line);

        /* Row 4: TX ping counter */
        snprintf(line, sizeof(line), "TX 0x100 cnt:%d", (int)g_tx_counter);
        u8g2_DrawStr(&u8g2, 0, 50, line);

        /* Row 5: RX frame count */
        snprintf(line, sizeof(line), "RX frames:%lu", (unsigned long)rx_count);
        u8g2_DrawStr(&u8g2, 0, 62, line);

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

    /* Enable error/state alerts only — exclude RX_DATA to avoid per-frame spam */
    twai_reconfigure_alerts(
        TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL |
        TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ARB_LOST,
        NULL);

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

/* ══════════════════════════════════════════════
 *  statsTask — every 3 seconds dump everything
 * ═════════════════════════════════════════════ */
static void statsTask(void *arg)
{
    (void)arg;
    twai_status_info_t st;
    uint32_t prev_total = 0, prev_accel = 0, prev_gyro = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        twai_get_status_info(&st);
        uint32_t d_total = g_n_total - prev_total;
        uint32_t d_accel = g_n_accel - prev_accel;
        uint32_t d_gyro  = g_n_gyro  - prev_gyro;
        prev_total = g_n_total; prev_accel = g_n_accel; prev_gyro = g_n_gyro;

        ESP_LOGW(TAG,
            "=== STATS ===  RX total=%lu (+%lu/3s)  accel=%lu (+%lu)  gyro=%lu (+%lu)  telem=%lu  ack=%lu  echo=%lu  other=%lu(last=0x%lX)",
            g_n_total, d_total, g_n_accel, d_accel, g_n_gyro, d_gyro,
            g_n_telem, g_n_ack, g_n_echo, g_n_other, g_last_other_id);
        ESP_LOGW(TAG, "=== STATS ===  BLE connected=%d",
            (int)ble_robot_connected());
        ESP_LOGW(TAG,
            "=== STATS ===  TWAI state=%d  tx_err=%lu  rx_err=%lu  tx_fail=%lu  rx_miss=%lu  rx_qlen=%lu  msgs_to_tx=%lu",
            st.state, st.tx_error_counter, st.rx_error_counter,
            st.tx_failed_count, st.rx_missed_count,
            st.msgs_to_rx, st.msgs_to_tx);
        ESP_LOGW(TAG,
            "=== STATS ===  IMU latest A=%d,%d,%d  G=%d,%d,%d  free_heap=%lu",
            g_ax, g_ay, g_az, g_gx, g_gy, g_gz,
            (unsigned long)esp_get_free_heap_size());
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
    ESP_LOGI(TAG, "=== MCU1 CAN + OLED + WiFi AP ===");

    /* NVS flash (required by WiFi driver) */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    oled_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(can_init());

    /* Start BLE GATT peripheral */
    ESP_ERROR_CHECK(ble_robot_init());

    /* Start WiFi AP + HTTP/WS server */
    wifi_ap_ws_init();

    oledQueue = xQueueCreate(1, sizeof(oled_msg_t));
    configASSERT(oledQueue);

    xTaskCreate(canRxTask,    "canRx",    4096, NULL, 3, NULL);
    /* canTxTask removed — was sending unnecessary 0x100 test pings to MCU3 */
    xTaskCreate(oledTask,     "oled",     4096, NULL, 1, NULL);
    xTaskCreate(canAlertTask, "canAlert", 2048, NULL, 1, NULL);
    xTaskCreate(statsTask,    "stats",    3072, NULL, 1, NULL);

    ESP_LOGI(TAG, "All tasks created");
}