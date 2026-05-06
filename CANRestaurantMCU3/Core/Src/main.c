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
// PA6  = TRIG1 (unused — sensor 1 disabled)
// PA7  = ECHO1 (unused)
// PA4  = TRIG2 (unused)
// PA1  = ECHO2 (unused)
// PA5  = TRIG3 FRONT  (GPIO output)
// PB0  = ECHO3 FRONT  (TIM3 CH3 input capture, AF2) <-- the only active sensor
// PB8  = TRIG4 (unused — sensor 4 disabled)
// PB1  = ECHO4 (unused)
// PB10 = FDCAN1 TX (AF9)
// PB12 = FDCAN1 RX (AF9)
// PC13 = LED blink
// ============================================================
#define TRIG1_PORT              GPIOA
#define TRIG1_PIN               GPIO_PIN_6

#define TRIG2_PORT              GPIOA
#define TRIG2_PIN               GPIO_PIN_4

#define TRIG3_PORT              GPIOA
#define TRIG3_PIN               GPIO_PIN_5

#define TRIG4_PORT              GPIOB
#define TRIG4_PIN               GPIO_PIN_8

// TIM2 & TIM3 @ 1MHz (Prescaler = 250-1 at 250MHz)
#define SONAR_PERIOD_US         20000
#define TRIG_PULSE_US           10
#define US_TO_CM                58.0f
#define ECHO_TIMEOUT_US         38000

// Obstacle threshold (cm) — front-only emergency stop
#define OBSTACLE_THRESHOLD_CM   15.0f

// ============================================================
// CAN MESSAGE IDs
// ============================================================
#define CAN_ID_OBSTACLE_ALERT   0x301  // MCU3 → MCU1
#define CAN_ID_EMERGENCY_STOP   0x302  // MCU3 → MCU2 (direct, bypass queue)
#define CAN_ID_PATH_CLEAR       0x303  // MCU3 → MCU1

// ============================================================
// EVENT GROUP BITS
// ============================================================
#define EMERGENCY_STOP_BIT      ( 1 << 0 )

// ============================================================
// HANDLES & GLOBALS
// ============================================================
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
FDCAN_HandleTypeDef hfdcan1;

static SemaphoreHandle_t printMutex = NULL;
static SemaphoreHandle_t stopActiveSem = NULL;
static SemaphoreHandle_t canTxMailboxSem = NULL;

// One task handle per sensor — ISR notifies the right task
static TaskHandle_t sonarHandle[4];  // [0]=Front [1]=Right [2]=Left [3]=Rear

// One single-slot queue per sensor
static QueueHandle_t obstacleQueue[4];
static QueueHandle_t canTxQueue = NULL;

static EventGroupHandle_t threatEventGroup = NULL;

// CAN message struct
typedef struct {
	uint32_t id;
	uint8_t data[8];
} CAN_MsgTypeDef;

// ============================================================
// ISR SHARED VOLATILE — per sensor
// ============================================================
// Sensor 1 Front — TIM2 CH3
static volatile uint32_t echo1Start = 0, echo1End = 0;
static volatile uint8_t cap1State = 0;

// Sensor 2 Right — TIM2 CH2
static volatile uint32_t echo2Start = 0, echo2End = 0;
static volatile uint8_t cap2State = 0;

// Sensor 3 Left — TIM3 CH3
static volatile uint32_t echo3Start = 0, echo3End = 0;
static volatile uint8_t cap3State = 0;

// Sensor 4 Rear — TIM3 CH4
static volatile uint32_t echo4Start = 0, echo4End = 0;
static volatile uint8_t cap4State = 0;

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
static void MX_TIM3_Init(void);
static void MX_FDCAN1_Init(void);

// ============================================================
// FDCAN TX FIFO EMPTY CALLBACK
// ============================================================
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
	if (hfdcan->Instance != FDCAN1)
		return;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(canTxMailboxSem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// DEBUG counters — read from tasks
static volatile uint32_t dbg_ch2IsrCount = 0;  // incremented in ISR

// ============================================================
// TIM2 OC CALLBACK — CH1 triggers sensors 1 & 2 simultaneously
// TIM3 OC CALLBACK — CH1 triggers sensors 3 & 4 simultaneously
// ============================================================
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		static uint8_t trigState2 = 0;
		if (trigState2 == 0) {
			HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_SET);
			trigState2 = 1;
			uint32_t nextCCR = __HAL_TIM_GET_COUNTER(htim) + TRIG_PULSE_US;
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, nextCCR);
		} else {
			HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET);
			trigState2 = 0;
			uint32_t nextCCR = __HAL_TIM_GET_COMPARE(htim,
					TIM_CHANNEL_1) + SONAR_PERIOD_US;
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, nextCCR);
		}
	}

	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		static uint8_t trigState3 = 0;
		if (trigState3 == 0) {
			HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TRIG4_PORT, TRIG4_PIN, GPIO_PIN_SET);
			trigState3 = 1;
			uint32_t nextCCR = __HAL_TIM_GET_COUNTER(htim) + TRIG_PULSE_US;
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, nextCCR);
		} else {
			HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TRIG4_PORT, TRIG4_PIN, GPIO_PIN_RESET);
			trigState3 = 0;
			uint32_t nextCCR = __HAL_TIM_GET_COMPARE(htim,
					TIM_CHANNEL_1) + SONAR_PERIOD_US;
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, nextCCR);
		}
	}
}

// ============================================================
// IC CALLBACKS
// TIM2 CH3 = Front, TIM2 CH2 = Right
// TIM3 CH3 = Left,  TIM3 CH4 = Rear
// ============================================================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Sensor 1 Front — TIM2 CH3
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (cap1State == 0) {
			echo1Start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_FALLING);
			cap1State = 1;
		} else {
			echo1End = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_RISING);
			cap1State = 0;
			vTaskNotifyGiveFromISR(sonarHandle[0], &xHigherPriorityTaskWoken);
		}
	}

	// Sensor 2 Right — TIM2 CH2
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {

		// DEBUG: confirm ISR is reached at all
		static uint32_t ch2IsrCount = 0;
		ch2IsrCount++;
		// Note: avoid printf in ISR — set a flag instead
		// We use a volatile counter, printed from the sonar task

		if (cap2State == 0) {
			echo2Start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
					TIM_INPUTCHANNELPOLARITY_FALLING);
			cap2State = 1;
		} else {
			echo2End = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
					TIM_INPUTCHANNELPOLARITY_RISING);
			cap2State = 0;
			vTaskNotifyGiveFromISR(sonarHandle[1], &xHigherPriorityTaskWoken);
		}
	}

	// Sensor 3 Left — TIM3 CH3
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (cap3State == 0) {
			echo3Start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_FALLING);
			cap3State = 1;
		} else {
			echo3End = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_RISING);
			cap3State = 0;
			vTaskNotifyGiveFromISR(sonarHandle[2], &xHigherPriorityTaskWoken);
		}
	}

	// Sensor 4 Rear — TIM3 CH4
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		if (cap4State == 0) {
			echo4Start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_FALLING);
			cap4State = 1;
		} else {
			echo4End = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_RISING);
			cap4State = 0;
			vTaskNotifyGiveFromISR(sonarHandle[3], &xHigherPriorityTaskWoken);
		}
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================
// HELPER: compute distance
// TIM2 = 32-bit, TIM3 = 16-bit
// ============================================================
static float computeDist(uint32_t start, uint32_t end, uint8_t is16bit) {
	uint32_t pulseWidth;
	if (end >= start) {
		pulseWidth = end - start;
	} else {
		uint32_t wrap = is16bit ? 0xFFFFU : 0xFFFFFFFFU;
		pulseWidth = (wrap - start) + end + 1U;
	}
	if (pulseWidth == 0 || pulseWidth > ECHO_TIMEOUT_US)
		return 999.0f;
	return (float) pulseWidth / US_TO_CM;
}

// ============================================================
// TASK 1-4: SONAR TASKS
// ============================================================
static void sonarTask1(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(200));
	PRINT("[MCU3] sonarTask1 Front started\r\n");
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		float dist = computeDist(echo1Start, echo1End, 0);
		xQueueOverwrite(obstacleQueue[0], &dist);
	}
}

static void sonarTask2(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(200));
	PRINT("[MCU3] sonarTask2 Right started\r\n");
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		float dist = computeDist(echo2Start, echo2End, 0);
		xQueueOverwrite(obstacleQueue[1], &dist);
	}
}

static void sonarTask3(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(200));
	PRINT("[MCU3] sonarTask3 Left started\r\n");
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		float dist = computeDist(echo3Start, echo3End, 1);
		xQueueOverwrite(obstacleQueue[2], &dist);
	}
}

static void sonarTask4(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(200));
	PRINT("[MCU3] sonarTask4 Rear started\r\n");
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		float dist = computeDist(echo4Start, echo4End, 1);
		xQueueOverwrite(obstacleQueue[3], &dist);
	}
}

// ============================================================
// TASK 5: OBSTACLE ANALYSIS  (FRONT-ONLY — sensor 3, PA5/PB0/TIM3 CH3)
// ============================================================
static void obstacleAnalysisTask(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(300));
	PRINT("[MCU3] Obstacle Analysis Task started (FRONT only, threshold=%.0fcm)\r\n",
			OBSTACLE_THRESHOLD_CM);

	static uint8_t wasEmergency = 0;
	float distFront = 999.0f;
	/* Drain other queues so they don't fill (sensors 1/2/4 may still publish). */
	float scrap;

	for (;;) {
		/* Block on FRONT sensor only — that's the obstacle source we trust. */
		xQueueReceive(obstacleQueue[2], &distFront, portMAX_DELAY);
		(void) xQueueReceive(obstacleQueue[0], &scrap, 0);
		(void) xQueueReceive(obstacleQueue[1], &scrap, 0);
		(void) xQueueReceive(obstacleQueue[3], &scrap, 0);

		PRINT("[sonar] FRONT=%.1f cm\r\n", distFront);

		uint8_t obstacle = (distFront < OBSTACLE_THRESHOLD_CM) ? 1 : 0;

		if (obstacle && !wasEmergency) {
			wasEmergency = 1;
			xEventGroupSetBits(threatEventGroup, EMERGENCY_STOP_BIT);
			PRINT("[MCU3] FRONT obstacle %.1fcm -> EMERGENCY_STOP_BIT\r\n",
					distFront);

		} else if (!obstacle && wasEmergency) {
			wasEmergency = 0;
			xSemaphoreGive(stopActiveSem);

			CAN_MsgTypeDef msg = { 0 };
			msg.id = CAN_ID_PATH_CLEAR;
			msg.data[0] = 0xC1;
			xQueueSend(canTxQueue, &msg, 0);
			PRINT("[MCU3] Front clear (%.1fcm) -> 0x303 PATH_CLEAR\r\n",
					distFront);
		}
	}
}

// ============================================================
// TASK 6: EMERGENCY STOP — highest priority
// ============================================================
static void emergencyStopTask(void *arg) {
	(void) arg;

	for (;;) {
		PRINT("[MCU3] emergTask: attempting 0x302 TX\r\n");

		xEventGroupWaitBits(threatEventGroup,
		EMERGENCY_STOP_BIT,
		pdTRUE, pdFALSE, portMAX_DELAY);

		if (xSemaphoreTake(stopActiveSem, 0) != pdTRUE) {
			PRINT("[MCU3] Stop already active, suppressing duplicate\r\n");
			continue;
		}

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
			xSemaphoreGive(stopActiveSem);
		}

		CAN_MsgTypeDef alertMsg = { 0 };
		alertMsg.id = CAN_ID_OBSTACLE_ALERT;
		alertMsg.data[0] = 0xA1;
		xQueueSend(canTxQueue, &alertMsg, 0);
	}
}

// ============================================================
// TASK 7: CAN TX
// ============================================================
static void canTxTask(void *arg) {
	(void) arg;
	PRINT("[MCU3] CAN Transmit Task started\r\n");

	CAN_MsgTypeDef msg;
	for (;;) {
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

		xSemaphoreTake(canTxMailboxSem, portMAX_DELAY);

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, msg.data)
				== HAL_OK) {
			PRINT("[MCU3] CAN TX OK  id=0x%03lX\r\n", msg.id);
		} else {
			PRINT("[MCU3] CAN TX FAILED id=0x%03lX err=%lu\r\n", msg.id,
					hfdcan1.ErrorCode);
			xSemaphoreGive(canTxMailboxSem);
		}
	}
}

// ============================================================
// TASK 8: LED BLINK
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
	MX_TIM3_Init();
	MX_FDCAN1_Init();

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		Error_Handler();
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0);

	// TIM2 — sensors 1 (Front) & 2 (Right)
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

	// TIM3 — sensors 3 (Left) & 4 (Rear)
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

	printMutex = xSemaphoreCreateMutex();
	stopActiveSem = xSemaphoreCreateBinary();
	canTxMailboxSem = xSemaphoreCreateBinary();
	threatEventGroup = xEventGroupCreate();

	for (int i = 0; i < 4; i++) {
		obstacleQueue[i] = xQueueCreate(1, sizeof(float));
	}
	canTxQueue = xQueueCreate(8, sizeof(CAN_MsgTypeDef));

	xSemaphoreGive(stopActiveSem);
	xSemaphoreGive(canTxMailboxSem);

	printf("[MCU3] System init OK\r\n");

	xTaskCreate(gpioTask, "gpioTask", 128, NULL, 1, NULL);
	xTaskCreate(sonarTask1, "sonar1", 256, NULL, 2, &sonarHandle[0]);
	xTaskCreate(sonarTask2, "sonar2", 256, NULL, 2, &sonarHandle[1]);
	xTaskCreate(sonarTask3, "sonar3", 256, NULL, 2, &sonarHandle[2]);
	xTaskCreate(sonarTask4, "sonar4", 256, NULL, 2, &sonarHandle[3]);
	xTaskCreate(obstacleAnalysisTask, "obsTask", 384, NULL, 3, NULL);
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
// TIM2 INIT — 1MHz, OC CH1 trigger, IC CH2+CH3 echo
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
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();

	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

// ============================================================
// TIM3 INIT — 1MHz, OC CH1 trigger, IC CH3+CH4 echo
// ============================================================
static void MX_TIM3_Init(void) {
	__HAL_RCC_TIM3_CLK_ENABLE();

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 250 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xFFFF;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
		Error_Handler();

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = SONAR_PERIOD_US;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	TIM_IC_InitTypeDef sConfigIC = { 0 };
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0x0F;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
		Error_Handler();

	HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// ============================================================
// GPIO INIT
// ============================================================
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// PC13 — LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// PA4=TRIG2 PA5=TRIG3 PA6=TRIG1 — GPIO outputs
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
			GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PA1 — TIM2 CH2 ECHO2 Right, AF1
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PA7 — TIM2 CH3 ECHO1 Front, AF1
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PB0 — TIM3 CH3 ECHO3 Left, AF2
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB1 — TIM3 CH4 ECHO4 Rear, AF2
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB8 — TRIG4 Rear, GPIO output
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB10 — FDCAN1 TX, AF9
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB12 — FDCAN1 RX, AF9
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
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
