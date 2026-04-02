#include "main.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

// ============================================================
// PIN DEFINITIONS — STEPPER (PA1=DIR, PA2=STEP)
// ============================================================
#define STEP_PORT           GPIOA
#define STEP_PIN            GPIO_PIN_2
#define DIR_PORT            GPIOA
#define DIR_PIN             GPIO_PIN_1

#define DIR_FORWARD         GPIO_PIN_SET
#define DIR_BACKWARD        GPIO_PIN_RESET

#define STEPS_PER_REV       200
#define STEPPER_SPEED_SLOW  2000
#define STEPPER_SPEED_MED   1000
#define STEPPER_SPEED_FAST  500

// ============================================================
// MPU6050
// ============================================================
#define MPU6050_ADDR        (0x68 << 1)
#define MPU6050_REG_PWR_MGT 0x6B
#define MPU6050_REG_ACCEL   0x3B

// ============================================================
// CAN MESSAGE IDs — MCU2 receiver
// ============================================================
#define CAN_ID_EMERGENCY_STOP   0x302  // MCU3 -> MCU2: stop immediately
#define CAN_ID_PATH_CLEAR       0x303  // MCU3 -> MCU1: path clear
#define CAN_ID_NAV_CMD          0x101  // MCU1 -> MCU2: navigation command

// ============================================================
// EVENT GROUP BITS
// ============================================================
#define EMERGENCY_STOP_BIT      ( 1 << 0 )  // Trigger to stop immediately
#define PATH_CLEAR_BIT          ( 1 << 1 )  // Trigger to resume
#define OBSTACLE_ACTIVE_BIT     ( 1 << 2 )  // State flag tracking if obstacle is present

// ============================================================
// CAN MESSAGE STRUCT
// ============================================================
typedef struct {
	uint32_t identifier;
	uint8_t data[8];
} CAN_MsgTypeDef;

// ============================================================
// HANDLES & GLOBALS
// ============================================================
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
FDCAN_HandleTypeDef hfdcan1;

static SemaphoreHandle_t printMutex = NULL;
static TaskHandle_t mpuTaskHandle = NULL;
static TaskHandle_t stepTaskHandle = NULL;

static QueueHandle_t canRxQueue = NULL;
static EventGroupHandle_t emergencyGroup = NULL;

// Stepper step counter — modified in ISR
static volatile uint32_t stepRemaining = 0;

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
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(uint32_t period);
static void MX_FDCAN1_Init(void);

// ============================================================
// FDCAN RX FIFO0 CALLBACK — ISR context
// ============================================================
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
		return;

	CAN_MsgTypeDef msg;
	FDCAN_RxHeaderTypeDef rxHeader;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, msg.data)
			== HAL_OK) {
		msg.identifier = rxHeader.Identifier;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(canRxQueue, &msg, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	HAL_FDCAN_ActivateNotification(hfdcan,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

// ============================================================
// TIM2 PERIOD ELAPSED CALLBACK — stepper pulse generation
// ============================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance != TIM2)
		return;

	static uint8_t pinState = 0;

	if (stepRemaining == 0) {
		HAL_TIM_Base_Stop_IT(&htim2);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(stepTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

// ============================================================
// MPU6050 EXTI CALLBACK — PB2
// ============================================================
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_2) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(mpuTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

// ============================================================
// STEPPER DRIVER
// ============================================================
typedef enum {
	STEPPER_FORWARD,
	STEPPER_BACKWARD,
	STEPPER_TURN_LEFT,
	STEPPER_TURN_RIGHT,
	STEPPER_STOP
} StepperCmd;

static void stepper_start(GPIO_PinState dir, uint32_t steps, uint32_t speed) {
	if (steps == 0)
		return;
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir);
	stepRemaining = steps;
	MX_TIM2_Init(speed);
	HAL_TIM_Base_Start_IT(&htim2);
}

void stepper_forward(uint32_t steps, uint32_t speed) {
	stepper_start(DIR_FORWARD, steps, speed);
}

void stepper_backward(uint32_t steps, uint32_t speed) {
	stepper_start(DIR_BACKWARD, steps, speed);
}

void stepper_turn_left(uint32_t steps, uint32_t speed) {
	stepper_start(DIR_BACKWARD, steps, speed);
}

void stepper_turn_right(uint32_t steps, uint32_t speed) {
	stepper_start(DIR_FORWARD, steps, speed);
}

void stepper_stop(void) {
	HAL_TIM_Base_Stop_IT(&htim2);
	stepRemaining = 0;
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
}

// stepper_run — starts motor then blocks task until TIM2 ISR done
void stepper_run(StepperCmd cmd, uint32_t steps, uint32_t speed) {
	switch (cmd) {
	case STEPPER_FORWARD:
		stepper_forward(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_BACKWARD:
		stepper_backward(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_TURN_LEFT:
		stepper_turn_left(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_TURN_RIGHT:
		stepper_turn_right(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_STOP:
		stepper_stop();
		break;
	}
}

// ============================================================
// TASK 1: CAN RX TASK
// Blocks on canRxQueue → routes messages to event group
// ============================================================
static void canRxTask(void *arg) {
	(void) arg;

	if (HAL_FDCAN_ActivateNotification(&hfdcan1,
	FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		PRINT("[canRx] Activate notification FAILED\r\n");
	}
	PRINT("[canRx] Task started, listening on CAN bus\r\n");

	CAN_MsgTypeDef msg;
	for (;;) {
		xQueueReceive(canRxQueue, &msg, portMAX_DELAY);

		switch (msg.identifier) {
		case CAN_ID_EMERGENCY_STOP:
			PRINT("[MCU2] RX 0x302 EMERGENCY_STOP from MCU3\r\n");
			// Set both the trigger and the active state flag
			xEventGroupSetBits(emergencyGroup,
			EMERGENCY_STOP_BIT | OBSTACLE_ACTIVE_BIT);
			break;

		case CAN_ID_PATH_CLEAR:
			PRINT("[MCU2] RX 0x303 PATH_CLEAR from MCU3\r\n");
			// Clear the state flag, set the clear trigger
			xEventGroupClearBits(emergencyGroup, OBSTACLE_ACTIVE_BIT);
			xEventGroupSetBits(emergencyGroup, PATH_CLEAR_BIT);
			break;

		case CAN_ID_NAV_CMD:
			PRINT("[MCU2] RX 0x101 NAV_CMD from MCU1\r\n");
			break;

		default:
			PRINT("[MCU2] RX unknown id=0x%03lX\r\n", msg.identifier);
			break;
		}
	}
}

// ============================================================
// TASK 2: EMERGENCY HANDLER TASK — highest priority on MCU2
// Blocks on event group → stops motor on EMERGENCY_STOP_BIT
// Sets PATH_CLEAR_BIT for stepperTask to resume on clear
// ============================================================
static void emergencyHandlerTask(void *arg) {
	(void) arg;

	for (;;) {
		// Wait ONLY for the EMERGENCY trigger
		xEventGroupWaitBits(emergencyGroup,
		EMERGENCY_STOP_BIT,
		pdTRUE,     // auto-clear bit on exit
				pdFALSE,
				portMAX_DELAY);

		stepper_stop();
		PRINT("[MCU2] EMERGENCY STOP! Motor halted\r\n");

		// CRITICAL FIX: Since TIM2 is stopped, the ISR will never fire to
		// unblock stepper_run(). We must manually wake the stepperTask.
		if (stepTaskHandle != NULL) {
			xTaskNotifyGive(stepTaskHandle);
		}
	}
}

// ============================================================
// TASK 3: BLINK TASK
// ============================================================
static void gpioTask(void *arg) {
	(void) arg;
	for (;;) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

// ============================================================
// TASK 4: MPU6050 TASK — woken by INT on PB2
// ============================================================
static void mpuTask(void *arg) {
	(void) arg;

	vTaskDelay(pdMS_TO_TICKS(100));

	uint8_t wakeup[2] = { MPU6050_REG_PWR_MGT, 0x00 };
	if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, wakeup, 2, 100)
			!= HAL_OK) {
		PRINT("[mpuTask] Wake up FAILED! err=%lu\r\n", hi2c1.ErrorCode);
		vTaskDelete(NULL);
		return;
	}
	PRINT("[mpuTask] MPU6050 woken up OK\r\n");
	vTaskDelay(pdMS_TO_TICKS(100));

	uint8_t accel_cfg[2] = { 0x1C, 0x00 };
	uint8_t gyro_cfg[2] = { 0x1B, 0x00 };
	uint8_t dlpf_cfg[2] = { 0x1A, 0x04 };
	uint8_t smplrt[2] = { 0x19, 0x13 };
	uint8_t int_cfg[2] = { 0x37, 0x10 };
	uint8_t int_enable[2] = { 0x38, 0x01 };

	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, accel_cfg, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, gyro_cfg, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, dlpf_cfg, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, smplrt, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, int_cfg, 2, 100);
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, int_enable, 2, 100);

	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	PRINT("[mpuTask] MPU6050 configured, waiting for INT on PB2...\r\n");

	uint8_t buf[14];
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		uint8_t reg = MPU6050_REG_ACCEL;
		if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100)
				!= HAL_OK) {
			PRINT("[mpuTask] TX reg FAILED! err=%lu\r\n", hi2c1.ErrorCode);
			continue;
		}
		if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100)
				!= HAL_OK) {
			PRINT("[mpuTask] RX FAILED! err=%lu\r\n", hi2c1.ErrorCode);
			continue;
		}

		int16_t ax = (int16_t) (buf[0] << 8 | buf[1]);
		int16_t ay = (int16_t) (buf[2] << 8 | buf[3]);
		int16_t az = (int16_t) (buf[4] << 8 | buf[5]);
		int16_t gx = (int16_t) (buf[8] << 8 | buf[9]);
		int16_t gy = (int16_t) (buf[10] << 8 | buf[11]);
		int16_t gz = (int16_t) (buf[12] << 8 | buf[13]);

		PRINT("[MPU6050] Accel: X=%.2f g  Y=%.2f g  Z=%.2f g  |"
				"  Gyro: X=%.2f  Y=%.2f  Z=%.2f deg/s\r\n",
				(float )ax / 16384.0f, (float )ay / 16384.0f,
				(float )az / 16384.0f, (float )gx / 131.0f, (float )gy / 131.0f,
				(float )gz / 131.0f);
	}
}

// ============================================================
// TASK 5: STEPPER TASK
// Runs motor continuously FORWARD
// Blocks on PATH_CLEAR_BIT when emergency occurs
// No polling — fully event driven
// ============================================================
static void stepperTask(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(500));
	PRINT("[stepper] Task started - running continuously FORWARD\r\n");

	for (;;) {
		// Run 200 steps — blocks until TIM2 ISR OR emergencyHandlerTask notifies done
		stepper_run(STEPPER_FORWARD, 200, STEPPER_SPEED_MED);

		// Check the dedicated state bit. It will safely remain 1 until MCU3 sends 0x303.
		if (xEventGroupGetBits(emergencyGroup) & OBSTACLE_ACTIVE_BIT) {
			PRINT("[stepper] Emergency detected - waiting for path clear\r\n");

			// Block here until the path clears
			xEventGroupWaitBits(emergencyGroup,
			PATH_CLEAR_BIT,
			pdTRUE,         // clear PATH_CLEAR_BIT on exit
					pdFALSE,
					portMAX_DELAY   // sleep forever until bit is set
					);

			PRINT("[stepper] Resuming\r\n");
		}
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
	MX_I2C1_Init();
	MX_TIM2_Init(STEPPER_SPEED_MED);
	MX_FDCAN1_Init();

	// FDCAN filter — accept all standard frames
	FDCAN_FilterTypeDef sFilterConfig = { 0 };
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
		Error_Handler();
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
	FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
		Error_Handler();
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		Error_Handler();

	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

	// Create synchronization primitives
	printMutex = xSemaphoreCreateMutex();
	emergencyGroup = xEventGroupCreate();
	canRxQueue = xQueueCreate(10, sizeof(CAN_MsgTypeDef));

	printf("[MCU2] System init OK\r\n");

	xTaskCreate(gpioTask, "gpioTask", 128, NULL, 1, NULL);
	xTaskCreate(stepperTask, "stepperTask", 256, NULL, 2, &stepTaskHandle);
	xTaskCreate(mpuTask, "mpuTask", 256, NULL, 3, &mpuTaskHandle);
	xTaskCreate(canRxTask, "canRxTask", 256, NULL, 4, NULL);
	xTaskCreate(emergencyHandlerTask, "emergTask", 256, NULL, 5, NULL);

	vTaskStartScheduler();
	while (1) {
	}
}

// ============================================================
// FDCAN1 INIT
// PB9 = TX (AF9), PB8 = RX (AF9)
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

// ============================================================
// I2C1 INIT
// ============================================================
static void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x60808CD3;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		Error_Handler();
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
		Error_Handler();
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
		Error_Handler();
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

	// PA1 (DIR) + PA2 (STEP) - A4988
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PB5 (SDA) + PB6 (SCL) - I2C1 AF4
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB2 - MPU6050 INT rising edge
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 6, 0);

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
