/*
 * mcu3_can.c — Minimal FDCAN TX/RX  (fixed)
 *
 * Fix: rxHdr.DataLength is a plain byte count (e.g. 8, 1),
 *      NOT the shifted FDCAN_DLC_BYTES_x constant.
 *      So: msg.len = (uint8_t)rxHdr.DataLength  — no shift.
 *
 * TX: sends "HiMCU3!!" (8 bytes) every 1 second, ID = 0x300
 * RX: accepts ALL standard IDs, prints ID + ASCII payload
 *
 * Pins:  PB10 = FDCAN1 TX (AF9)
 *        PB12 = FDCAN1 RX (AF9)
 * Baud:  1 Mbps @ 250 MHz  (Prescaler=25 Seg1=8 Seg2=1)
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define MCU3_TX_ID      0x300
#define MCU3_TX_PERIOD  pdMS_TO_TICKS(1000)

FDCAN_HandleTypeDef hfdcan1;

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} CAN_Msg_t;

static QueueHandle_t rxQueue = NULL;

/* ═══════════════════════════════════════════════════════════════
 * FDCAN RX FIFO0 CALLBACK  (ISR context)
 * ═══════════════════════════════════════════════════════════════ */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
		return;

	CAN_Msg_t msg = { 0 };
	FDCAN_RxHeaderTypeDef rxHdr;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHdr, msg.data)
			== HAL_OK) {
		msg.id = rxHdr.Identifier;
		msg.len = (uint8_t) rxHdr.DataLength; /* plain byte count, no shift */
	}

	BaseType_t woken = pdFALSE;
	xQueueSendFromISR(rxQueue, &msg, &woken);
	portYIELD_FROM_ISR(woken);

	HAL_FDCAN_ActivateNotification(hfdcan,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 1 — CAN TX  (every 1 s)
 * ═══════════════════════════════════════════════════════════════ */
static void canTxTask(void *arg) {
	(void) arg;

	const char payload[9] = "HiMCU3!!"; /* [9] = 8 chars + null terminator */

	FDCAN_TxHeaderTypeDef txHdr = { 0 };
	txHdr.Identifier = MCU3_TX_ID;
	txHdr.IdType = FDCAN_STANDARD_ID;
	txHdr.TxFrameType = FDCAN_DATA_FRAME;
	txHdr.DataLength = FDCAN_DLC_BYTES_8;
	txHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHdr.BitRateSwitch = FDCAN_BRS_OFF;
	txHdr.FDFormat = FDCAN_CLASSIC_CAN;
	txHdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHdr.MessageMarker = 0;

	for (;;) {
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHdr, (uint8_t*) payload)
				== HAL_OK)
			printf("[MCU3] TX  id=0x%03X  data=\"%.8s\"\r\n",
			MCU3_TX_ID, payload);
		else
			printf("[MCU3] TX FAILED  err=%lu\r\n", hfdcan1.ErrorCode);

		vTaskDelay(MCU3_TX_PERIOD);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 2 — CAN RX
 * ═══════════════════════════════════════════════════════════════ */
static void canRxTask(void *arg) {
	(void) arg;

	CAN_Msg_t msg;
	for (;;) {
		xQueueReceive(rxQueue, &msg, portMAX_DELAY);

		printf("[MCU3] RX  id=0x%03lX  len=%u  data=\"", msg.id, msg.len);
		for (int i = 0; i < msg.len; i++) {
			uint8_t c = msg.data[i];
			printf("%c", (c >= 0x20 && c < 0x7F) ? c : '.');
		}
		printf("\"\r\n");
	}
}

/* ═══════════════════════════════════════════════════════════════
 * FDCAN1 INIT — 1 Mbps @ 250 MHz
 * ═══════════════════════════════════════════════════════════════ */
static void MX_FDCAN1_Init(void) {
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = ENABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	hfdcan1.Init.NominalPrescaler = 25;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 8;
	hfdcan1.Init.NominalTimeSeg2 = 1;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 1;
	hfdcan1.Init.DataTimeSeg1 = 1;
	hfdcan1.Init.DataTimeSeg2 = 1;
	hfdcan1.Init.StdFiltersNbr = 1;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
		Error_Handler();
}

/* ═══════════════════════════════════════════════════════════════
 * GPIO INIT — PB10=TX  PB12=RX  (AF9)
 * ═══════════════════════════════════════════════════════════════ */
static void MX_GPIO_Init(void) {
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef g = { 0 };
	g.Mode = GPIO_MODE_AF_PP;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = GPIO_AF9_FDCAN1;

	g.Pin = GPIO_PIN_10;
	g.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &g);

	g.Pin = GPIO_PIN_12;
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &g);
}

/* ═══════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════ */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_FDCAN1_Init();

	FDCAN_FilterTypeDef f = { 0 };
	f.IdType = FDCAN_STANDARD_ID;
	f.FilterIndex = 0;
	f.FilterType = FDCAN_FILTER_MASK;
	f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	f.FilterID1 = 0x000;
	f.FilterID2 = 0x000; /* mask=0 → accept all IDs */
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK)
		Error_Handler();

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
	FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE,
	FDCAN_FILTER_REMOTE) != HAL_OK)
		Error_Handler();

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		Error_Handler();

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	HAL_FDCAN_ActivateNotification(&hfdcan1,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	rxQueue = xQueueCreate(10, sizeof(CAN_Msg_t));

	printf("[MCU3] System init OK — CAN TX/RX ready\r\n");

	xTaskCreate(canTxTask, "canTx", 256, NULL, 2, NULL);
	xTaskCreate(canRxTask, "canRx", 256, NULL, 2, NULL);

	vTaskStartScheduler();
	while (1) {
	}
}
