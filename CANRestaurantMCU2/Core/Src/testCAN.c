/*
 * testCAN.c — MCU2 FDCAN TX/RX test with debug
 *
 * TX: sends "HiMCU2!!" every 1 s, ID = 0x200
 * RX: accepts ALL standard IDs, prints ID + hex + ASCII
 * DBG: every 2 s prints TX/RX counters + FDCAN PSR/ECR
 *
 * Pins:  PB10 = FDCAN1 TX (AF9)
 *        PB12 = FDCAN1 RX (AF9)
 *        PC13 = LED heartbeat
 * Baud:  1 Mbps @ 250 MHz  (Prescaler=25 Seg1=8 Seg2=1)
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define MCU2_TX_ID      0x200
#define MCU2_TX_PERIOD  pdMS_TO_TICKS(1000)
#define DBG_PERIOD      pdMS_TO_TICKS(2000)

FDCAN_HandleTypeDef hfdcan1;

/* Stub handles — required by stm32h5xx_it.c IRQ handlers */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} CAN_Msg_t;

static QueueHandle_t rxQueue = NULL;
static SemaphoreHandle_t printMutex = NULL;

#define SAFE_PRINTF(...) do { \
	if (printMutex) xSemaphoreTake(printMutex, portMAX_DELAY); \
	printf(__VA_ARGS__); \
	if (printMutex) xSemaphoreGive(printMutex); \
} while (0)

/* -- Debug counters ---- */
static volatile uint32_t g_txCount = 0;
static volatile uint32_t g_rxCount = 0;
static volatile uint32_t g_txFail = 0;
static volatile uint32_t g_isrCount = 0;
static volatile uint32_t g_errCount = 0;

/* Forward declarations */
static void MX_FDCAN1_Init(void);
static void MX_GPIO_Init(void);
void SystemClock_Config(void);
void Error_Handler(void);

/* ═══════════════════════════════════════════════════════════════
 * FDCAN RX FIFO0 CALLBACK  (ISR context)
 * ═══════════════════════════════════════════════════════════════ */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
		return;

	g_isrCount++;

	CAN_Msg_t msg = { 0 };
	FDCAN_RxHeaderTypeDef rxHdr;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHdr, msg.data)
			== HAL_OK) {
		msg.id = rxHdr.Identifier;
		msg.len = (uint8_t) rxHdr.DataLength; /* HAL stores plain byte count */
		g_rxCount++;
	}

	BaseType_t woken = pdFALSE;
	xQueueSendFromISR(rxQueue, &msg, &woken);
	portYIELD_FROM_ISR(woken);

	HAL_FDCAN_ActivateNotification(hfdcan,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ═══════════════════════════════════════════════════════════════
 * FDCAN ERROR CALLBACK  (ISR context)
 * ═══════════════════════════════════════════════════════════════ */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
	g_errCount++;
	(void) hfdcan;
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 1 — CAN TX  (every 1 s)
 * ═══════════════════════════════════════════════════════════════ */
static void canTxTask(void *arg) {
	(void) arg;

	const char payload[9] = "HiMCU2!!"; /* [9] = 8 chars + null terminator */

	FDCAN_TxHeaderTypeDef txHdr = { 0 };
	txHdr.Identifier = MCU2_TX_ID;
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
				== HAL_OK) {
			g_txCount++;
			SAFE_PRINTF("[TX] id=0x%03X  #%lu  data=\"%-.8s\"\r\n", MCU2_TX_ID,
					g_txCount, payload);
		} else {
			g_txFail++;
			SAFE_PRINTF("[TX] FAILED #%lu  err=0x%08lX  PSR=0x%08lX\r\n",
					g_txFail, hfdcan1.ErrorCode, hfdcan1.Instance->PSR);
		}

		vTaskDelay(MCU2_TX_PERIOD);
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

		if (printMutex)
			xSemaphoreTake(printMutex, portMAX_DELAY);
		/* hex dump */
		printf("[RX] id=0x%03lX len=%u hex:", msg.id, msg.len);
		for (int i = 0; i < msg.len; i++)
			printf(" %02X", msg.data[i]);

		/* ASCII */
		printf("  ascii:\"");
		for (int i = 0; i < msg.len; i++) {
			uint8_t c = msg.data[i];
			printf("%c", (c >= 0x20 && c < 0x7F) ? c : '.');
		}
		printf("\"\r\n");
		if (printMutex)
			xSemaphoreGive(printMutex);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 3 — DEBUG: stats + PSR/ECR every 2 s
 * ═══════════════════════════════════════════════════════════════ */
static void dbgTask(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(500));
	SAFE_PRINTF("[DBG] task started\r\n");

	for (;;) {
		uint32_t psr = hfdcan1.Instance->PSR;
		uint32_t ecr = hfdcan1.Instance->ECR;
		uint8_t lec = psr & 0x7U;
		uint8_t ep = (psr >> 5) & 1U;
		uint8_t bo = (psr >> 6) & 1U;
		uint8_t tec = (ecr >> 16) & 0xFFU;
		uint8_t rec = ecr & 0x7FU;

		const char *lec_str[] = { "NoErr", "Stuff", "Form", "Ack", "Bit1",
				"Bit0", "CRC", "NoChg" };

		if (printMutex)
			xSemaphoreTake(printMutex, portMAX_DELAY);
		printf("[DBG] tx=%lu rx=%lu txFail=%lu isrHit=%lu errCB=%lu\r\n",
				g_txCount, g_rxCount, g_txFail, g_isrCount, g_errCount);
		printf(
				"[DBG] PSR=0x%08lX ECR=0x%08lX LEC=%s EP=%u BO=%u TEC=%u REC=%u\r\n",
				psr, ecr, lec_str[lec], ep, bo, tec, rec);

		if (bo)
			printf("[DBG] *** BUS-OFF! Check termination + wiring ***\r\n");
		else if (ep)
			printf("[DBG] *** Error-Passive (TEC/REC >= 128) ***\r\n");
		if (printMutex)
			xSemaphoreGive(printMutex);

		vTaskDelay(DBG_PERIOD);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 4 — LED blink PC13 @ 500 ms = alive
 * ═══════════════════════════════════════════════════════════════ */
static void ledTask(void *arg) {
	(void) arg;
	for (;;) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		vTaskDelay(pdMS_TO_TICKS(500));
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
 * GPIO INIT — PB10=TX  PB12=RX  (AF9) + PC13=LED
 * ═══════════════════════════════════════════════════════════════ */
static void MX_GPIO_Init(void) {
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

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

	/* PC13 — LED output */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	g.Pin = GPIO_PIN_13;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Alternate = 0;
	HAL_GPIO_Init(GPIOC, &g);
}

/* ═══════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════ */
int main(void) {
	HAL_Init();
	SystemClock_Config();

	/* ITM/SWO trace */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	ITM->TCR |= ITM_TCR_ITMENA_Msk;
	ITM->TER |= (1U << 0);

	MX_GPIO_Init();
	MX_FDCAN1_Init();

	FDCAN_FilterTypeDef f = { 0 };
	f.IdType = FDCAN_STANDARD_ID;
	f.FilterIndex = 0;
	f.FilterType = FDCAN_FILTER_MASK;
	f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	f.FilterID1 = 0x000;
	f.FilterID2 = 0x000; /* mask=0 -> accept all IDs */
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

	/* RX + error notifications */
	HAL_FDCAN_ActivateNotification(&hfdcan1,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF
					| FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
			0);

	rxQueue = xQueueCreate(10, sizeof(CAN_Msg_t));
	printMutex = xSemaphoreCreateMutex();

	printf("[MCU2] testCAN init OK — TX=0x200  RX=all  @1Mbps\r\n");
	printf("[MCU2] PSR=0x%08lX ECR=0x%08lX\r\n", hfdcan1.Instance->PSR,
			hfdcan1.Instance->ECR);

	xTaskCreate(canTxTask, "canTx", 256, NULL, 2, NULL);
	xTaskCreate(canRxTask, "canRx", 256, NULL, 2, NULL);
	xTaskCreate(dbgTask, "dbg", 512, NULL, 1, NULL);
	xTaskCreate(ledTask, "led", 128, NULL, 1, NULL);

	vTaskStartScheduler();
	while (1) {
	}
}

/* ══════════════════════════════════════════════
 *  Clock Config — CSI → PLL1 → 250 MHz SYSCLK
 *  (identical to original project)
 * ══════════════════════════════════════════════ */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
	RCC_OscInitStruct.CSIState = RCC_CSI_ON;
	RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 125;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
		Error_Handler();

	__HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* ══════════════════════════════════════════════ */
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
