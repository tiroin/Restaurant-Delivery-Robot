/**
 * @file    can_bus.c
 * @brief   CAN Bus Driver + CANTxTask + CANRxTask — Implementation
 */

#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "can_bus.h"

static const char *TAG = "CANBus";

/* ─────────────────────────────────────────────────────────────
 *  GPIO — override bằng sdkconfig (Kconfig)
 * ───────────────────────────────────────────────────────────── */
#ifndef CONFIG_CAN_TX_GPIO
#define CONFIG_CAN_TX_GPIO  21
#endif
#ifndef CONFIG_CAN_RX_GPIO
#define CONFIG_CAN_RX_GPIO  20
#endif

/* Timeout gửi frame (nếu TX queue đầy) */
#define CAN_TX_ENQUEUE_TIMEOUT_MS   100

/* ─────────────────────────────────────────────────────────────
 *  Queue & EventGroup definitions (extern trong can_bus.h)
 * ───────────────────────────────────────────────────────────── */
QueueHandle_t      xCANTxQueue    = NULL;
QueueHandle_t      xCANRxQueue    = NULL;
EventGroupHandle_t xEmergencyGroup = NULL;

/** Semaphore nội bộ: báo CAN mailbox rảnh */
static SemaphoreHandle_t s_mailbox_sem = NULL;

/* ════════════════════════════════════════════════════════════
 *  can_bus_init
 * ════════════════════════════════════════════════════════════ */
esp_err_t can_bus_init(void)
{
    /* Tạo FreeRTOS primitives */
    xCANTxQueue    = xQueueCreate(16, sizeof(twai_message_t));
    xCANRxQueue    = xQueueCreate(32, sizeof(twai_message_t));
    xEmergencyGroup = xEventGroupCreate();
    s_mailbox_sem  = xSemaphoreCreateBinary();

    configASSERT(xCANTxQueue);
    configASSERT(xCANRxQueue);
    configASSERT(xEmergencyGroup);
    configASSERT(s_mailbox_sem);

    xSemaphoreGive(s_mailbox_sem); /* Mailbox ban đầu rảnh */

    /* Cấu hình TWAI driver */
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CONFIG_CAN_TX_GPIO,
                                    (gpio_num_t)CONFIG_CAN_RX_GPIO,
                                    TWAI_MODE_NORMAL);
    g_cfg.rx_queue_len = 32;
    g_cfg.tx_queue_len = 16;

    twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "TWAI started — TX:GPIO%d  RX:GPIO%d  @1Mbps",
             CONFIG_CAN_TX_GPIO, CONFIG_CAN_RX_GPIO);
    return ESP_OK;
}

/* ════════════════════════════════════════════════════════════
 *  can_send_command  (public helper)
 * ════════════════════════════════════════════════════════════ */
esp_err_t can_send_command(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (len > 8) return ESP_ERR_INVALID_ARG;

    twai_message_t frame = {
        .identifier       = can_id,
        .data_length_code = len,
        .extd             = 0,
        .rtr              = 0,
        .ss               = 0,
        .self             = 0,
        .dlc_non_comp     = 0,
    };

    if (data && len > 0) {
        memcpy(frame.data, data, len);
    }

    if (xQueueSend(xCANTxQueue, &frame,
                   pdMS_TO_TICKS(CAN_TX_ENQUEUE_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "can_send_command: TX queue full (id=0x%03lX)", can_id);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/* ════════════════════════════════════════════════════════════
 *  CANTxTask
 *  Priority: 4  |  Stack: 4096
 * ════════════════════════════════════════════════════════════ */
void can_tx_task(void *pvParam)
{
    (void)pvParam;
    twai_message_t frame;
    ESP_LOGI(TAG, "CANTxTask started");

    for (;;) {
        /*
         * BLOCK: Task ngủ tại đây cho đến khi StateMachineTask
         * gọi can_send_command() → push frame vào xCANTxQueue.
         */
        xQueueReceive(xCANTxQueue, &frame, portMAX_DELAY);

        /*
         * BLOCK: Chờ mailbox rảnh trước khi load frame tiếp theo.
         * Semaphore được give ngay sau khi twai_transmit thành công,
         * đảm bảo không gửi frame mới khi bus vẫn đang truyền.
         */
        xSemaphoreTake(s_mailbox_sem, portMAX_DELAY);

        esp_err_t err = twai_transmit(&frame,
                                      pdMS_TO_TICKS(CAN_TX_ENQUEUE_TIMEOUT_MS));
        if (err == ESP_OK) {
            ESP_LOGD(TAG, "Tx → id=0x%03lX  len=%d  data[0]=0x%02X",
                     frame.identifier, 
                     frame.data_length_code,
                     frame.data_length_code > 0 ? frame.data[0] : 0);
        } else {
            ESP_LOGE(TAG, "twai_transmit failed id=0x%03lX: %s",
                     frame.identifier, esp_err_to_name(err));
        }

        /* UNBLOCK: release mailbox cho frame tiếp theo */
        xSemaphoreGive(s_mailbox_sem);

        /* Nhỏ throttle bảo vệ bus */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* ════════════════════════════════════════════════════════════
 *  CANRxTask
 *  Priority: 5  |  Stack: 4096
 * ════════════════════════════════════════════════════════════ */
void can_rx_task(void *pvParam)
{
    (void)pvParam;
    twai_message_t frame;
    ESP_LOGI(TAG, "CANRxTask started");

    for (;;) {
        /*
         * BLOCK: Task ngủ cho đến khi có frame xuất hiện trên CAN bus.
         * TWAI hardware interrupt fires → wakes task via driver notify.
         */
        esp_err_t err = twai_receive(&frame, portMAX_DELAY);
        if (err != ESP_OK) {
            if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "twai_receive err: %s", esp_err_to_name(err));
            }
            continue;
        }

        ESP_LOGD(TAG, "Rx ← id=0x%03lX  len=%d  data[0]=0x%02X",
                 frame.identifier,
                 frame.data_length_code,
                 frame.data_length_code > 0 ? frame.data[0] : 0);

        /*
         * EMERGENCY PATH — fastest possible path, no queuing.
         * Set event group bit ngay lập tức để StateMachineTask
         * (đang block trên xCANRxQueue hoặc xOrderQueue) được
         * unblock qua xEventGroupGetBits() ở vòng lặp kế tiếp.
         */
        if (frame.identifier == CAN_ID_EMERGENCY ||
            frame.identifier == CAN_ID_OBSTACLE) {

            EventBits_t bit = (frame.identifier == CAN_ID_EMERGENCY)
                              ? EMERGENCY_STOP_BIT
                              : OBSTACLE_BIT;

            ESP_LOGW(TAG, "EMERGENCY frame id=0x%03lX — set bit 0x%02X",
                     frame.identifier, (unsigned)bit);
            xEventGroupSetBits(xEmergencyGroup, bit);
            /* Fall through: vẫn push vào queue để StateMachine log */
        }

        /* Push vào xCANRxQueue (non-blocking: drop nếu đầy) */
        if (xQueueSend(xCANRxQueue, &frame, pdMS_TO_TICKS(20)) != pdTRUE) {
            ESP_LOGW(TAG, "xCANRxQueue full — frame id=0x%03lX dropped",
                     frame.identifier);
        }
    }
}
