#include "main.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

// ============================================================
// PIN DEFINITIONS
// PA6  = TRIG (GPIO output)
// PA7  = ECHO (TIM2 CH3 input capture, AF1)
// PB10 = FDCAN1 TX (AF9)
// PB12 = FDCAN1 RX (AF9)
// PC13 = LED blink
// ============================================================
#define TRIG_PORT               GPIOA
#define TRIG_PIN                GPIO_PIN_6

// TIM2 @ 1MHz
#define SONAR_PERIOD_US         20000
#define TRIG_PULSE_US           10
#define US_TO_CM                58.0f
#define ECHO_TIMEOUT_US         38000

// Obstacle threshold
#define OBSTACLE_THRESHOLD_CM   10.0f

// ============================================================
// CAN MESSAGE IDs
// ============================================================
#define CAN_ID_OBSTACLE_ALERT   0x301  // MCU3 → MCU1
#define CAN_ID_EMERGENCY_STOP   0x302  // MCU3 → MCU2 (direct, bypass queue)
#define CAN_ID_PATH_CLEAR       0x303  // MCU3 → MCU1

// ============================================================
// EVENT GROUP BITS
// ============================================================
#define EMERGENCY_STOP_BIT      ( 1 << 0 )  // bit 0 = obstacle detected

// ============================================================
// HANDLES & GLOBALS
// ============================================================
TIM_HandleTypeDef htim2;
FDCAN_HandleTypeDef hfdcan1;

static SemaphoreHandle_t printMutex = NULL;
static SemaphoreHandle_t stopActiveSem = NULL; // suppress duplicate stops
static SemaphoreHandle_t canTxMailboxSem = NULL; // mailbox 0/1 free signal
static TaskHandle_t ultrasonicHandle = NULL;

static QueueHandle_t obstacleQueue = NULL; // float distance
static QueueHandle_t canTxQueue = NULL; // CAN_MsgTypeDef

static EventGroupHandle_t threatEventGroup = NULL;

// CAN message struct
typedef struct {
	uint32_t id;
	uint8_t data[8];
} CAN_MsgTypeDef;

// Shared volatile between ISR and ultrasonicTask
static volatile uint32_t echoStart = 0;
static volatile uint32_t echoEnd = 0;
static volatile uint8_t capState = 0;

// ============================================================
// PRINT MACRO
// ============================================================
#define PRINT(...) do { \
    xSemaphoreTake(printMutex, portMAX_DELAY); \
    printf(__VA_ARGS__); \
    xSemaphoreGive(printMutex); \
} while(0)

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);

// ============================================================
// FDCAN TX COMPLETE CALLBACK
// Releases mailbox semaphore so canTxTask can send next frame
// ============================================================
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
	if (hfdcan->Instance != FDCAN1)
		return;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(canTxMailboxSem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================
// TIM2 OUTPUT COMPARE CALLBACK — CH1
// Fires every 20ms → generates 10us TRIG pulse
// ============================================================
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance != TIM2)
		return;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		static uint8_t trigState = 0;

		if (trigState == 0) {
			HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
			trigState = 1;

			uint32_t nextCCR = __HAL_TIM_GET_COUNTER(htim) + TRIG_PULSE_US;
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, nextCCR);

		} else {
			HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
			trigState = 0;

			uint32_t nextCCR = __HAL_TIM_GET_COMPARE(htim,
					TIM_CHANNEL_1) + SONAR_PERIOD_US;
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, nextCCR);

			if (capState == 0)
				capState = 0;
		}
	}
}

// ============================================================
// TIM2 INPUT CAPTURE CALLBACK — CH3
// Rising  → t_start
// Falling → t_end → notify ultrasonicTask
// ============================================================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance != TIM2)
		return;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (capState == 0) {
			echoStart = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_FALLING);
			capState = 1;
		} else {
			echoEnd = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_RISING);
			capState = 0;

			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(ultrasonicHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

// ============================================================
// TASK 1: ULTRASONIC SCAN TASK
// Woken by TIM2 IC ISR → computes distance → posts to obstacleQueue
// ============================================================
static void ultrasonicTask(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(200));
	PRINT("[MCU3] Ultrasonic Scan Task started\r\n");

	for (;;) {
		// Block until TIM2 IC falling edge ISR notifies
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		uint32_t start = echoStart;
		uint32_t end = echoEnd;

		uint32_t pulseWidth;
		if (end >= start) {
			pulseWidth = end - start;
		} else {
			pulseWidth = (0xFFFFFFFF - start) + end + 1;
		}

		float dist;
		if (pulseWidth == 0 || pulseWidth > ECHO_TIMEOUT_US) {
			dist = 999.0f;  // out of range
		} else {
			dist = (float) pulseWidth / US_TO_CM;
		}

		// Post to obstacleQueue — overwrites old value if not consumed yet
		// use xQueueOverwrite for single-slot queue (always fresh data)
		xQueueOverwrite(obstacleQueue, &dist);
	}
}

// ============================================================
// TASK 2: OBSTACLE ANALYSIS TASK
// Blocks on obstacleQueue → evaluates distance → sets event group
// or pushes PATH_CLEAR to canTxQueue
// ============================================================
static void obstacleAnalysisTask(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(300));
	PRINT("[MCU3] Obstacle Analysis Task started\r\n");

	float dist;
	static uint8_t wasEmergency = 0;

	for (;;) {
		// Block until ultrasonicTask posts a fresh distance
		xQueueReceive(obstacleQueue, &dist, portMAX_DELAY);

		PRINT("[sonar] Distance: %.1f cm\r\n", dist);

		if (dist < OBSTACLE_THRESHOLD_CM && !wasEmergency) {
			// Transition to EMERGENCY
			wasEmergency = 1;

			// Set emergency stop bit — wakes Emergency Stop Task immediately
			xEventGroupSetBits(threatEventGroup, EMERGENCY_STOP_BIT);
			PRINT("[MCU3] Obstacle! Setting EMERGENCY_STOP_BIT\r\n");

		} else if (dist >= OBSTACLE_THRESHOLD_CM && wasEmergency) {
			// Transition back to CLEAR
			wasEmergency = 0;

			// Release stopActiveSem so next emergency can send 0x302 again
			xSemaphoreGive(stopActiveSem);

			// Push PATH_CLEAR frame to CAN Transmit Task queue
			CAN_MsgTypeDef msg = { 0 };
			msg.id = CAN_ID_PATH_CLEAR;
			msg.data[0] = 0xC1;
			xQueueSend(canTxQueue, &msg, 0);
			PRINT("[MCU3] Path clear - pushing 0x303 to canTxQueue\r\n");
		}
	}
}

// ============================================================
// TASK 3: EMERGENCY STOP TASK — highest priority on MCU3
// Blocks on event group bit → sends 0x302 DIRECTLY to FDCAN
// bypassing canTxQueue for zero delay
// ============================================================
static void emergencyStopTask(void *arg) {
	(void) arg;

	for (;;) {
		// Block permanently until EMERGENCY_STOP_BIT is set
		xEventGroupWaitBits(threatEventGroup,
		EMERGENCY_STOP_BIT,
		pdTRUE,         // clear bit on exit
				pdFALSE,        // wait for any bit (only 1 bit here)
				portMAX_DELAY);

		// Check if a stop is already in flight — suppress duplicate
		if (xSemaphoreTake(stopActiveSem, 0) != pdTRUE) {
			// Semaphore already taken — stop already sent, skip
			PRINT("[MCU3] Stop already active, suppressing duplicate\r\n");
			continue;
		}

		// Write 0x302 DIRECTLY to FDCAN TX Mailbox — bypass all queues
		FDCAN_TxHeaderTypeDef txHeader = { 0 };
		txHeader.Identifier = CAN_ID_EMERGENCY_STOP;
		txHeader.IdType = FDCAN_STANDARD_ID;
		txHeader.TxFrameType = FDCAN_DATA_FRAME;
		txHeader.DataLength = FDCAN_DLC_BYTES_8;
		txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		txHeader.BitRateSwitch = FDCAN_BRS_OFF;
		txHeader.FDFormat = FDCAN_CLASSIC_CAN;
		txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		txHeader.MessageMarker = 0;

		uint8_t buf[8] = { 0xE5, 0, 0, 0, 0, 0, 0, 0 };

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, buf) == HAL_OK) {
			PRINT("[MCU3] 0x302 EMERGENCY_STOP sent directly -> MCU2\r\n");
		} else {
			PRINT("[MCU3] EMERGENCY_STOP TX FAILED! Err=%lu\r\n",
					hfdcan1.ErrorCode);
			// Release semaphore so next attempt can try
			xSemaphoreGive(stopActiveSem);
		}

		// Also alert MCU1 via normal queue
		CAN_MsgTypeDef alertMsg = { 0 };
		alertMsg.id = CAN_ID_OBSTACLE_ALERT;
		alertMsg.data[0] = 0xA1;
		xQueueSend(canTxQueue, &alertMsg, 0);
	}
}

// ============================================================
// TASK 4: CAN TRANSMIT TASK
// Blocks on canTxQueue → sends via Mailbox 0/1
// Never uses Mailbox 2 — reserved for Emergency Stop Task
// ============================================================
static void canTxTask(void *arg) {
	(void) arg;
	PRINT("[MCU3] CAN Transmit Task started\r\n");

	CAN_MsgTypeDef msg;

	for (;;) {
		// Block until a frame is queued
		xQueueReceive(canTxQueue, &msg, portMAX_DELAY);

		FDCAN_TxHeaderTypeDef txHeader = { 0 };
		txHeader.Identifier = msg.id;
		txHeader.IdType = FDCAN_STANDARD_ID;
		txHeader.TxFrameType = FDCAN_DATA_FRAME;
		txHeader.DataLength = FDCAN_DLC_BYTES_8;
		txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		txHeader.BitRateSwitch = FDCAN_BRS_OFF;
		txHeader.FDFormat = FDCAN_CLASSIC_CAN;
		txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		txHeader.MessageMarker = 0;

		// Wait for mailbox to be free
		xSemaphoreTake(canTxMailboxSem, portMAX_DELAY);

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, msg.data)
				== HAL_OK) {
			PRINT("[MCU3] CAN TX OK  id=0x%03lX\r\n", msg.id);
		} else {
			PRINT("[MCU3] CAN TX FAILED id=0x%03lX err=%lu\r\n", msg.id,
					hfdcan1.ErrorCode);
			// Give back semaphore since TX didn't actually use mailbox
			xSemaphoreGive(canTxMailboxSem);
		}
	}
}

// ============================================================
// TASK 5: BLINK TASK
// ============================================================
static void gpioTask(void *arg) {
	(void) arg;
	for (;;) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

// ============================================================
// MAIN
// ============================================================
int main(void) {
	HAL_Init();

	ITM->TCR |= (1U << ITM_TCR_ITMENA_Pos);
	ITM->TER |= (1U << 0);

	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_FDCAN1_Init();

	// FDCAN setup ...
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		Error_Handler();
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0);

	// TIM2 start
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

	// ← Create mutex FIRST before any PRINT
	printMutex = xSemaphoreCreateMutex();
	stopActiveSem = xSemaphoreCreateBinary();
	canTxMailboxSem = xSemaphoreCreateBinary();
	threatEventGroup = xEventGroupCreate();
	obstacleQueue = xQueueCreate(1, sizeof(float));
	canTxQueue = xQueueCreate(8, sizeof(CAN_MsgTypeDef));

	xSemaphoreGive(stopActiveSem);
	xSemaphoreGive(canTxMailboxSem);

	// NOW safe to use printf directly (before scheduler)
	printf("[MCU3] System init OK\r\n");

	xTaskCreate(gpioTask, "gpioTask", 128, NULL, 1, NULL);
	xTaskCreate(ultrasonicTask, "sonarTask", 256, NULL, 2, &ultrasonicHandle);
	xTaskCreate(obstacleAnalysisTask, "obsTask", 256, NULL, 3, NULL);
	xTaskCreate(canTxTask, "canTxTask", 256, NULL, 3, NULL);
	xTaskCreate(emergencyStopTask, "emergTask", 256, NULL, 5, NULL);

	vTaskStartScheduler();
	while (1) {
	}
}

// ============================================================
// FDCAN1 INIT
// ============================================================
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

// ============================================================
// TIM2 INIT
// ============================================================
static void MX_TIM2_Init(void) {
	__HAL_RCC_TIM2_CLK_ENABLE();

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 250 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFFFFFF;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		Error_Handler();

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = SONAR_PERIOD_US;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	TIM_IC_InitTypeDef sConfigIC = { 0 };
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0x0F;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();

	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

// ============================================================
// GPIO INIT
// ============================================================
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// PC13 - LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// PA6 - TRIG
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = TRIG_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

	// PA7 - ECHO TIM2 CH3 AF1
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PB10 - FDCAN1 TX AF9
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB12 - FDCAN1 RX AF9
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// ============================================================
// CLOCK CONFIG
// ============================================================
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

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
