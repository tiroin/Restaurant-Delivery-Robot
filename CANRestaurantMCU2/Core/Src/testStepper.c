/*
 * mcu2_stepper.c — Simple dual stepper control only
 *
 * Stepper 1:  PA1=DIR   PA2=STEP   TIM2
 * Stepper 2:  PA8=DIR2  PA9=STEP2  TIM3
 *
 * Both steppers run FORWARD continuously, 200 steps at a time.
 * TIM2/TIM3 generate the step pulses via period-elapsed ISR.
 * Each stepper task blocks on ulTaskNotifyTake() until its
 * batch of steps is done, then immediately starts the next.
 *
 * Speed constants (TIM period in timer ticks @ Prescaler=249):
 *   SLOW = 2000   MED = 1000   FAST = 500
 *
 * Clock: 250 MHz sysclk, Prescaler=249 → timer tick = 1 µs
 *   So period=1000 → 1 step pulse every 1 ms → ~500 steps/s
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ── Pin definitions ────────────────────────────────────────── */
#define STEP_PORT       GPIOA
#define STEP_PIN        GPIO_PIN_2
#define DIR_PORT        GPIOA
#define DIR_PIN         GPIO_PIN_1

#define STEP2_PORT      GPIOA
#define STEP2_PIN       GPIO_PIN_9
#define DIR2_PORT       GPIOA
#define DIR2_PIN        GPIO_PIN_8

/* Stepper 1 — left side */
#define DIR1_FORWARD    GPIO_PIN_SET
#define DIR1_BACKWARD   GPIO_PIN_RESET

/* Stepper 2 — right side (physically mirrored, so polarity is inverted) */
#define DIR2_FORWARD    GPIO_PIN_RESET
#define DIR2_BACKWARD   GPIO_PIN_SET

/* ── Speed presets (timer period in µs) ─────────────────────── */
#define STEPPER_SPEED_CRAWL 4000   /* 125 steps/s — heavy load */
#define STEPPER_SPEED_SLOW  2000
#define STEPPER_SPEED_MED   STEPPER_SPEED_SLOW
#define STEPPER_SPEED_FAST   500
#define STEPPER_SPEED_MAX    250   /* 2000 steps/s */

/* ── CAN ─────────────────────────────────────────────────────── */
/* Manual move: data[0]=dir(0-3), data[1:2]=steps BE16, data[3:4]=speed_us BE16 */
#define CAN_ID_MANUAL_MOVE  0x110U   /* ESP32 → MCU2 */

typedef struct {
	uint32_t id;
	uint8_t  data[8];
	uint8_t  len;
} CAN_Msg_t;

/* ── Handles ────────────────────────────────────────────────── */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
FDCAN_HandleTypeDef hfdcan1;   /* required by stm32h5xx_it.c */

static SemaphoreHandle_t printMutex = NULL;
static TaskHandle_t stepTask1 = NULL;
static TaskHandle_t stepTask2 = NULL;
static QueueHandle_t canRxQueue = NULL;

/* ── Latest nav command (set by canNavTask, read by stepper tasks) ── */
static volatile uint8_t  g_navDir   = 0;    /* 0=FWD 1=BWD 2=LEFT 3=RIGHT */
static volatile uint16_t g_navSteps = 0;    /* 0 = STOP */
static volatile uint16_t g_navSpeed = STEPPER_SPEED_CRAWL;

/* ── Step counters (written by task, decremented by ISR) ─────── */
static volatile uint32_t stepRemaining = 0;
static volatile uint32_t stepRemaining2 = 0;

/* ── Print helper ───────────────────────────────────────────── */
#define PRINT(...) do { \
    xSemaphoreTake(printMutex, portMAX_DELAY); \
    printf(__VA_ARGS__); \
    xSemaphoreGive(printMutex); \
} while(0)

/* ═══════════════════════════════════════════════════════════════
 * TIM2 / TIM3 PERIOD ELAPSED ISR
 *
 * Each callback toggles the STEP pin and decrements the counter.
 * One full step = two timer periods (rising + falling edge).
 * When counter reaches 0 the timer stops and notifies the task.
 * ═══════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* ── Stepper 1 — TIM2 ── */
	if (htim->Instance == TIM2) {
		static uint8_t pinState = 0;

		if (stepRemaining == 0) {
			HAL_TIM_Base_Stop_IT(&htim2);
			BaseType_t woken = pdFALSE;
			vTaskNotifyGiveFromISR(stepTask1, &woken);
			portYIELD_FROM_ISR(woken);
			return;
		}

		if (pinState == 0) {
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			pinState = 1;
		} else {
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			pinState = 0;
			stepRemaining--;
		}
	}

	/* ── Stepper 2 — TIM3 ── */
	if (htim->Instance == TIM3) {
		static uint8_t pinState2 = 0;

		if (stepRemaining2 == 0) {
			HAL_TIM_Base_Stop_IT(&htim3);
			BaseType_t woken = pdFALSE;
			vTaskNotifyGiveFromISR(stepTask2, &woken);
			portYIELD_FROM_ISR(woken);
			return;
		}

		if (pinState2 == 0) {
			HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_SET);
			pinState2 = 1;
		} else {
			HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_RESET);
			pinState2 = 0;
			stepRemaining2--;
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * STEPPER 1 DRIVER  (TIM2)
 * ═══════════════════════════════════════════════════════════════ */
static void MX_TIM2_Init(uint32_t period) {
	__HAL_RCC_TIM2_CLK_ENABLE();
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 249;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = period - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		Error_Handler();
}

static void stepper1_run(GPIO_PinState dir, uint32_t steps, uint32_t speed) {
	if (steps == 0)
		return;
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir);
	stepRemaining = steps;
	MX_TIM2_Init(speed);
	HAL_TIM_Base_Start_IT(&htim2);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* block until done */
}

static void stepper1_stop(void) {
	HAL_TIM_Base_Stop_IT(&htim2);
	stepRemaining = 0;
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════
 * STEPPER 2 DRIVER  (TIM3)
 * ═══════════════════════════════════════════════════════════════ */
static void MX_TIM3_Init(uint32_t period) {
	__HAL_RCC_TIM3_CLK_ENABLE();
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 249;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = period - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
		Error_Handler();
}

static void stepper2_run(GPIO_PinState dir, uint32_t steps, uint32_t speed) {
	if (steps == 0)
		return;
	HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, dir);
	stepRemaining2 = steps;
	MX_TIM3_Init(speed);
	HAL_TIM_Base_Start_IT(&htim3);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* block until done */
}

static void stepper2_stop(void) {
	HAL_TIM_Base_Stop_IT(&htim3);
	stepRemaining2 = 0;
	HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 1 — STEPPER 1  TEST: run FORWARD continuously
 * ═══════════════════════════════════════════════════════════════ */
static void stepperTask1(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(500));
	PRINT("[stepper1] TEST — running FORWARD continuously\r\n");

	for (;;) {
		stepper1_run(DIR1_FORWARD, 500, STEPPER_SPEED_CRAWL);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 2 — STEPPER 2  TEST: run FORWARD continuously
 * ═══════════════════════════════════════════════════════════════ */
static void stepperTask2(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(500));
	PRINT("[stepper2] TEST — running FORWARD continuously\r\n");

	for (;;) {
		stepper2_run(DIR2_FORWARD, 500, STEPPER_SPEED_CRAWL);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * BLINK TASK — PC13 heartbeat so you know the scheduler is alive
 * ═══════════════════════════════════════════════════════════════ */
static void blinkTask(void *arg) {
	(void) arg;
	for (;;) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/* ═══════════════════════════════════════════════════════════════
 * GPIO INIT
 * PA1=DIR1  PA2=STEP1  PA8=DIR2  PA9=STEP2  PC13=LED
 * ═══════════════════════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════════════════════
 * FDCAN1 INIT — 1 Mbps @ 250 MHz
 * PB10 = FDCAN1 TX (AF9)   PB12 = FDCAN1 RX (AF9)
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

static void MX_GPIO_Init(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef g = { 0 };

	/* PA1, PA2, PA8, PA9 — stepper outputs */
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
	g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &g);

	/* PC13 — LED */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	g.Pin = GPIO_PIN_13;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &g);

	/* PB10 = FDCAN1 TX  AF9 */
	g.Pin = GPIO_PIN_10;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &g);

	/* PB12 = FDCAN1 RX  AF9 */
	g.Pin = GPIO_PIN_12;
	g.Pull = GPIO_PULLUP;
	g.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &g);
}

/* ═══════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════ */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM2_Init(STEPPER_SPEED_CRAWL);
	MX_TIM3_Init(STEPPER_SPEED_CRAWL);
	MX_FDCAN1_Init();

	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	/* FDCAN filter — accept all standard IDs into RX FIFO 0 */
	FDCAN_FilterTypeDef f = { 0 };
	f.IdType = FDCAN_STANDARD_ID;
	f.FilterIndex = 0;
	f.FilterType = FDCAN_FILTER_MASK;
	f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	f.FilterID1 = 0x000;
	f.FilterID2 = 0x000;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
			FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_Start(&hfdcan1);

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	HAL_FDCAN_ActivateNotification(&hfdcan1,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF
			| FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR, 0);

	printMutex = xSemaphoreCreateMutex();
	canRxQueue = xQueueCreate(1, sizeof(CAN_Msg_t));  /* depth=1: keep latest */

	printf("[MCU2] CAN-stepper init OK  RX=0x110 @1Mbps\r\n");

	xTaskCreate(blinkTask,    "blink",    128, NULL, 1, NULL);
	xTaskCreate(stepperTask1, "stepper1", 256, NULL, 2, &stepTask1);
	xTaskCreate(stepperTask2, "stepper2", 256, NULL, 2, &stepTask2);
	/* canNavTask not started — TEST MODE: motors run forward continuously */

	vTaskStartScheduler();
	while (1) {
	}
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
