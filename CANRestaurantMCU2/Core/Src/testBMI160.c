#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "flash_storage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

// ============================================================
// PIN DEFINITIONS — STEPPER 1 (PA1=DIR, PA2=STEP)
// ============================================================
#define STEP_PORT           GPIOA
#define STEP_PIN            GPIO_PIN_2
#define DIR_PORT            GPIOA
#define DIR_PIN             GPIO_PIN_1

// ============================================================
// PIN DEFINITIONS — STEPPER 2 (PA8=DIR2, PA9=STEP2)
// ============================================================
#define STEP2_PORT          GPIOA
#define STEP2_PIN           GPIO_PIN_9
#define DIR2_PORT           GPIOA
#define DIR2_PIN            GPIO_PIN_8

#define DIR_FORWARD         GPIO_PIN_SET
#define DIR_BACKWARD        GPIO_PIN_RESET

#define STEPS_PER_REV       200
#define STEPPER_SPEED_SLOW  2000
#define STEPPER_SPEED_MED   1000
#define STEPPER_SPEED_FAST  500

// ============================================================
// BMI160 DEFINITIONS
// ============================================================
#define BMI160_ADDR                 (0x68 << 1) // Change to 0x69<<1 if SA0 is pulled high
#define BMI160_REG_CHIP_ID          0x00
#define BMI160_REG_PMU_STATUS       0x03
#define BMI160_REG_DATA_START       0x0C        // Start of Gyro X
#define BMI160_REG_INT_EN_1         0x51
#define BMI160_REG_INT_OUT_CTRL     0x53
#define BMI160_REG_INT_MAP_1        0x56
#define BMI160_REG_CMD              0x7E

#define BMI160_CMD_SOFT_RESET       0xB6
#define BMI160_CMD_ACC_NORMAL       0x11
#define BMI160_CMD_GYR_NORMAL       0x15

#define BMI160_CHIP_ID_VAL          0xD1

// ============================================================
// CAN MESSAGE IDs
// ============================================================
#define CAN_ID_OBSTACLE_ALERT   0x301
#define CAN_ID_EMERGENCY_STOP   0x302
#define CAN_ID_PATH_CLEAR       0x303
#define CAN_ID_NAV_CMD          0x101

/* MCU1 → MCU2 (teach-and-replay) */
#define CAN_ID_MANUAL_MOVE      0x110U  /* dir, steps BE16, speed BE16 */
#define CAN_ID_SAVE_CP          0x111U  /* data[0] = checkpoint id     */
#define CAN_ID_CLEAR_CP         0x112U  /* data[0] = checkpoint id     */
#define CAN_ID_SET_MODE         0x113U  /* data[0] = 0/1/2             */

/* MCU2 → MCU1 (telemetry) */
#define CAN_ID_TELEMETRY        0x210U
#define CAN_ID_IMU_DATA         0x211U
#define CAN_ID_CP_SAVED_ACK     0x212U

/* Mode constants */
#define MCU2_MODE_IDLE   0
#define MCU2_MODE_LEARN  1
#define MCU2_MODE_AUTO   2

/* Move direction constants */
#define MOVE_DIR_FWD   0
#define MOVE_DIR_BWD   1
#define MOVE_DIR_LEFT  2
#define MOVE_DIR_RIGHT 3

// ============================================================
// EVENT GROUP BITS
// ============================================================
#define EMERGENCY_STOP_BIT      ( 1 << 0 )
#define PATH_CLEAR_BIT          ( 1 << 1 )
#define OBSTACLE_ACTIVE_BIT     ( 1 << 2 )

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
TIM_HandleTypeDef htim3;
FDCAN_HandleTypeDef hfdcan1;

static SemaphoreHandle_t printMutex = NULL;
static SemaphoreHandle_t i2cMutex = NULL;
static TaskHandle_t bmiTaskHandle = NULL;
static TaskHandle_t stepTaskHandle = NULL;   // stepper 1
static TaskHandle_t stepTaskHandle2 = NULL;  // stepper 2

static QueueHandle_t canRxQueue = NULL;
static EventGroupHandle_t emergencyGroup = NULL;

// Stepper 1 — decremented in TIM2 ISR
static volatile uint32_t stepRemaining = 0;
// Stepper 2 — decremented in TIM3 ISR
static volatile uint32_t stepRemaining2 = 0;

/* Teach-and-replay state */
static volatile int32_t g_steps_l = 0; /* cumulative left  steps */
static volatile int32_t g_steps_r = 0; /* cumulative right steps */
static volatile int32_t g_heading_x10 = 0; /* heading ×10, degrees  */
static volatile uint8_t g_mode = MCU2_MODE_IDLE;
static volatile uint8_t g_last_cp = 0xFF;

/* Latest raw BMI160 data for telemetry */
static volatile int16_t g_ax = 0, g_ay = 0, g_gz = 0;

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
static void MX_TIM3_Init(uint32_t period);
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
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

// ============================================================
// TIM2 PERIOD ELAPSED CALLBACK — stepper 1 & 2 pulse generation
// ============================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	// --- Stepper 1: TIM2 ---
	if (htim->Instance == TIM2) {
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

	// --- Stepper 2: TIM3 ---
	if (htim->Instance == TIM3) {
		static uint8_t pinState2 = 0;

		if (stepRemaining2 == 0) {
			HAL_TIM_Base_Stop_IT(&htim3);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(stepTaskHandle2, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

// ============================================================
// EXTI CALLBACKS — PB2 (BMI160 INT1)
// ============================================================
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (GPIO_Pin == GPIO_PIN_2) {
		if (bmiTaskHandle != NULL) {
			vTaskNotifyGiveFromISR(bmiTaskHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

// ============================================================
// STEPPER DRIVER — STEPPER 1 (TIM2)
// ============================================================
typedef enum {
	STEPPER_FORWARD,
	STEPPER_BACKWARD,
	STEPPER_TURN_LEFT,
	STEPPER_TURN_RIGHT,
	STEPPER_STOP
} StepperCmd;

/* ============================================================
 * CAN TX HELPER
 * ============================================================ */
static HAL_StatusTypeDef can_tx(uint32_t id, const uint8_t *data, uint8_t len) {
	FDCAN_TxHeaderTypeDef txHdr = { .Identifier = id, .IdType =
			FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME, .DataLength =
			(uint32_t) len << 16U, /* FDCAN_DLC_BYTES_x */
	.ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
			.FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl =
					FDCAN_NO_TX_EVENTS, .MessageMarker = 0, };
	/* Map plain byte count to FDCAN DLC field */
	const uint32_t dlc_table[] = {
	FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
	FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
	FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8, };
	if (len <= 8U) {
		txHdr.DataLength = dlc_table[len];
	} else {
		txHdr.DataLength = FDCAN_DLC_BYTES_8;
	}
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHdr, data);
}

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
// STEPPER DRIVER — STEPPER 2 (TIM3)
// ============================================================
static void stepper2_start(GPIO_PinState dir, uint32_t steps, uint32_t speed) {
	if (steps == 0)
		return;
	HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, dir);
	stepRemaining2 = steps;
	MX_TIM3_Init(speed);
	HAL_TIM_Base_Start_IT(&htim3);
}

void stepper2_forward(uint32_t steps, uint32_t speed) {
	stepper2_start(DIR_FORWARD, steps, speed);
}
void stepper2_backward(uint32_t steps, uint32_t speed) {
	stepper2_start(DIR_BACKWARD, steps, speed);
}
void stepper2_turn_left(uint32_t steps, uint32_t speed) {
	stepper2_start(DIR_BACKWARD, steps, speed);
}
void stepper2_turn_right(uint32_t steps, uint32_t speed) {
	stepper2_start(DIR_FORWARD, steps, speed);
}

void stepper2_stop(void) {
	HAL_TIM_Base_Stop_IT(&htim3);
	stepRemaining2 = 0;
	HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_RESET);
}

void stepper2_run(StepperCmd cmd, uint32_t steps, uint32_t speed) {
	switch (cmd) {
	case STEPPER_FORWARD:
		stepper2_forward(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_BACKWARD:
		stepper2_backward(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_TURN_LEFT:
		stepper2_turn_left(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_TURN_RIGHT:
		stepper2_turn_right(steps, speed);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		break;
	case STEPPER_STOP:
		stepper2_stop();
		break;
	}
}

// ============================================================
// TASK 1: CAN RX TASK
// ============================================================
static void canRxTask(void *arg) {
	(void) arg;

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		PRINT("[canRx] Activate notification FAILED\r\n");
	}
	PRINT("[canRx] Task started, listening on CAN bus\r\n");

	CAN_MsgTypeDef msg;
	for (;;) {
		xQueueReceive(canRxQueue, &msg, portMAX_DELAY);

		switch (msg.identifier) {
		case CAN_ID_OBSTACLE_ALERT:
			break;
		case CAN_ID_EMERGENCY_STOP:
			PRINT("[MCU2] RX 0x302 EMERGENCY_STOP from MCU3\r\n");
			xEventGroupSetBits(emergencyGroup,
			EMERGENCY_STOP_BIT | OBSTACLE_ACTIVE_BIT);
			break;
		case CAN_ID_PATH_CLEAR:
			PRINT("[MCU2] RX 0x303 PATH_CLEAR from MCU3\r\n");
			xEventGroupClearBits(emergencyGroup, OBSTACLE_ACTIVE_BIT);
			xEventGroupSetBits(emergencyGroup, PATH_CLEAR_BIT);
			break;
		case CAN_ID_NAV_CMD:
			PRINT("[MCU2] RX 0x101 NAV_CMD from MCU1\r\n");
			break;
		case 0x105:
			PRINT("[MCU2] RX 0x105 HEARTBEAT from MCU1, count=%u\r\n",
					msg.data[0]);
			break;

			/* ---- Teach-and-replay commands from MCU1 ---- */
		case CAN_ID_SET_MODE: {
			uint8_t new_mode = msg.data[0];
			if (new_mode <= MCU2_MODE_AUTO) {
				g_mode = new_mode;
				PRINT("[MCU2] Mode -> %u\r\n", new_mode);
			}
			break;
		}
		case CAN_ID_MANUAL_MOVE: {
			/* data[0]=dir, [1:2]=steps BE16, [3:4]=speed BE16 */
			uint8_t dir = msg.data[0];
			uint16_t steps = ((uint16_t) msg.data[1] << 8) | msg.data[2];
			uint16_t speed = ((uint16_t) msg.data[3] << 8) | msg.data[4];
			if (speed == 0)
				speed = STEPPER_SPEED_MED;
			PRINT("[MCU2] MANUAL_MOVE dir=%u steps=%u speed=%u\r\n", dir, steps,
					speed);
			switch (dir) {
			case MOVE_DIR_FWD:
				stepper_forward(steps, speed);
				stepper2_forward(steps, speed);
				g_steps_l += steps;
				g_steps_r += steps;
				break;
			case MOVE_DIR_BWD:
				stepper_backward(steps, speed);
				stepper2_backward(steps, speed);
				g_steps_l -= steps;
				g_steps_r -= steps;
				break;
			case MOVE_DIR_LEFT:
				stepper_turn_left(steps, speed);
				stepper2_turn_right(steps, speed);
				g_steps_l -= steps;
				g_steps_r += steps;
				g_heading_x10 -= (int32_t) steps; /* approx */
				break;
			case MOVE_DIR_RIGHT:
				stepper_turn_right(steps, speed);
				stepper2_turn_left(steps, speed);
				g_steps_l += steps;
				g_steps_r -= steps;
				g_heading_x10 += (int32_t) steps; /* approx */
				break;
			default:
				break;
			}
			break;
		}
		case CAN_ID_SAVE_CP: {
			uint8_t cp_id = msg.data[0];
			uint8_t ack[2] = { cp_id, 0x01U }; /* default: FAIL */
			if (cp_id < FLASH_STORAGE_MAX_CP) {
				HAL_StatusTypeDef st = flash_storage_save_cp(cp_id,
						(int32_t) g_steps_l, (int32_t) g_steps_r,
						(int32_t) g_heading_x10);
				if (st == HAL_OK) {
					g_last_cp = cp_id;
					ack[1] = 0x00U; /* OK */
					PRINT("[MCU2] CP %u saved OK sl=%ld sr=%ld h=%ld\r\n",
							cp_id, (long )g_steps_l, (long )g_steps_r,
							(long )g_heading_x10);
				} else {
					PRINT("[MCU2] CP %u save FAILED\r\n", cp_id);
				}
			}
			can_tx(CAN_ID_CP_SAVED_ACK, ack, sizeof(ack));
			break;
		}
		case CAN_ID_CLEAR_CP: {
			flash_storage_erase_all();
			g_steps_l = 0;
			g_steps_r = 0;
			g_heading_x10 = 0;
			g_last_cp = 0xFF;
			PRINT("[MCU2] All checkpoints cleared\r\n");
			break;
		}

		default:
			PRINT("[MCU2] RX unknown id=0x%03lX\r\n", msg.identifier);
			break;
		}
	}
}

// ============================================================
// TASK 2: EMERGENCY HANDLER TASK
// ============================================================
static void emergencyHandlerTask(void *arg) {
	(void) arg;

	for (;;) {
		xEventGroupWaitBits(emergencyGroup, EMERGENCY_STOP_BIT, pdTRUE, pdFALSE,
		portMAX_DELAY);

		stepper_stop();
		stepper2_stop();
		PRINT("[MCU2] EMERGENCY STOP! Both motors halted\r\n");

		if (stepTaskHandle != NULL)
			xTaskNotifyGive(stepTaskHandle);
		if (stepTaskHandle2 != NULL)
			xTaskNotifyGive(stepTaskHandle2);
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
// TASK 4: BMI160 HARDWARE INTERRUPT TASK
// ============================================================
static void bmi160Task(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(200)); // Wait for power to stabilize

	uint8_t id = 0;
	uint8_t cmd = 0;
	// Check I2C bus state
	PRINT("[BMI160] I2C ErrorCode=0x%08lX\r\n", hi2c1.ErrorCode);
	PRINT("[BMI160] I2C State=0x%02X\r\n", hi2c1.State);

	// Try scanning the bus — check both possible BMI160 addresses
	uint8_t dummy;
	HAL_StatusTypeDef s1 = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x00, 1,
			&dummy, 1, 100);
	HAL_StatusTypeDef s2 = HAL_I2C_Mem_Read(&hi2c1, (0x69 << 1), 0x00, 1,
			&dummy, 1, 100);
	PRINT("[BMI160] Probe 0x68=%d  0x69=%d  (0=ACK/found, 1=NACK/absent)\r\n",
			s1, s2);
	xSemaphoreTake(i2cMutex, portMAX_DELAY);

	// 1. Read Chip ID
	HAL_I2C_Mem_Read(&hi2c1, BMI160_ADDR, BMI160_REG_CHIP_ID, 1, &id, 1, 100);
	if (id != BMI160_CHIP_ID_VAL) {
		PRINT("[BMI160] Initialization FAILED. Incorrect Chip ID.\r\n");
		xSemaphoreGive(i2cMutex);
		vTaskDelete(NULL);
		return;
	}

	// 2. Soft Reset
	cmd = BMI160_CMD_SOFT_RESET;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
	xSemaphoreGive(i2cMutex);
	vTaskDelay(pdMS_TO_TICKS(15)); // Wait for reset

	xSemaphoreTake(i2cMutex, portMAX_DELAY);
	// 3. Power up Accelerometer to Normal Mode
	cmd = BMI160_CMD_ACC_NORMAL;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
	xSemaphoreGive(i2cMutex);
	vTaskDelay(pdMS_TO_TICKS(15));

	xSemaphoreTake(i2cMutex, portMAX_DELAY);
	// 4. Power up Gyroscope to Normal Mode
	cmd = BMI160_CMD_GYR_NORMAL;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);

	// 5. Configure Interrupts for Data Ready (DRDY) on INT1 Pin (PB2)
	// Enable INT1 Output, Active High, Push-Pull
	cmd = 0x0A;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_INT_OUT_CTRL, 1, &cmd, 1,
			100);

	// Map Data Ready Interrupt to INT1 Pin
	cmd = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_INT_MAP_1, 1, &cmd, 1,
			100);

	// Enable Data Ready Interrupt globally
	cmd = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_INT_EN_1, 1, &cmd, 1,
			100);

	xSemaphoreGive(i2cMutex);

	PRINT(
			"[BMI160] Initialized in Normal Mode with DRDY Interrupt on PB2.\r\n");

	uint8_t rawData[12];
	uint8_t printCounter = 0;

	for (;;) {
		// Wait indefinitely until the hardware EXTI callback gives the notification
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		xSemaphoreTake(i2cMutex, portMAX_DELAY);
		HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, BMI160_ADDR,
		BMI160_REG_DATA_START, 1, rawData, 12, 100);
		xSemaphoreGive(i2cMutex);

		if (ret != HAL_OK)
			continue;

		// Combine Little Endian bytes
		int16_t gx = (int16_t) ((rawData[1] << 8) | rawData[0]);
		int16_t gy = (int16_t) ((rawData[3] << 8) | rawData[2]);
		int16_t gz = (int16_t) ((rawData[5] << 8) | rawData[4]);

		int16_t ax = (int16_t) ((rawData[7] << 8) | rawData[6]);
		int16_t ay = (int16_t) ((rawData[9] << 8) | rawData[8]);
		int16_t az = (int16_t) ((rawData[11] << 8) | rawData[10]);

		/* Update global telemetry snapshot */
		g_ax = ax;
		g_ay = ay;
		g_gz = gz;
		(void) gx;
		(void) gy;
		(void) az;

		// Print every 10th reading to avoid flooding the console (100Hz / 10 = 10Hz print rate)
		if (++printCounter >= 10) {
			printCounter = 0;

			float accel_x = (float) ax / 16384.0f;
			float accel_y = (float) ay / 16384.0f;
			float accel_z = (float) az / 16384.0f;

			float gyro_x = (float) gx / 16.4f;
			float gyro_y = (float) gy / 16.4f;
			float gyro_z = (float) gz / 16.4f;

			PRINT(
					"[BMI160-INT] Accel: X=%.2fg Y=%.2fg Z=%.2fg | Gyro: X=%.2f Y=%.2f Z=%.2f deg/s\r\n",
					accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
		}
	}
}

// ============================================================
// TASK 5: TELEMETRY BROADCAST TASK (200 ms)
// Sends 0x210 (TELEMETRY) and 0x211 (IMU_DATA) to MCU1 every 200 ms
// ============================================================
static void telemetryTask(void *arg) {
	(void) arg;
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(200));

		/* --- 0x210 TELEMETRY: sl(2) sr(2) heading(2) mode(1) last_cp(1) --- */
		uint8_t tel[8];
		int32_t sl = g_steps_l;
		int32_t sr = g_steps_r;
		int32_t hd = g_heading_x10;
		tel[0] = (uint8_t) (sl >> 8);
		tel[1] = (uint8_t) (sl);
		tel[2] = (uint8_t) (sr >> 8);
		tel[3] = (uint8_t) (sr);
		tel[4] = (uint8_t) ((hd / 10) >> 8);
		tel[5] = (uint8_t) (hd / 10);
		tel[6] = g_mode;
		tel[7] = g_last_cp;
		can_tx(CAN_ID_TELEMETRY, tel, 8);

		/* --- 0x211 IMU_DATA: ax*100(2) ay*100(2) gz*10(2) --- */
		uint8_t imu[6];
		int16_t ax100 = (int16_t) ((int32_t) g_ax * 100 / 16384);
		int16_t ay100 = (int16_t) ((int32_t) g_ay * 100 / 16384);
		int16_t gz10 = (int16_t) ((int32_t) g_gz * 10 / 164);
		imu[0] = (uint8_t) (ax100 >> 8);
		imu[1] = (uint8_t) (ax100);
		imu[2] = (uint8_t) (ay100 >> 8);
		imu[3] = (uint8_t) (ay100);
		imu[4] = (uint8_t) (gz10 >> 8);
		imu[5] = (uint8_t) (gz10);
		can_tx(CAN_ID_IMU_DATA, imu, 6);
	}
}

// ============================================================
// TASK 6: STEPPER 1 TASK
// ============================================================
static void stepperTask(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(500));
	PRINT("[stepper1] Task started - running continuously FORWARD\r\n");

	for (;;) {
		stepper_run(STEPPER_FORWARD, 200, STEPPER_SPEED_MED);

		if (xEventGroupGetBits(emergencyGroup) & OBSTACLE_ACTIVE_BIT) {
			PRINT("[stepper1] Emergency detected - waiting for path clear\r\n");
			xEventGroupWaitBits(emergencyGroup, PATH_CLEAR_BIT, pdTRUE, pdFALSE,
			portMAX_DELAY);
			PRINT("[stepper1] Resuming\r\n");
		}
	}
}

// ============================================================
// TASK 7: STEPPER 2 TASK
// ============================================================
static void stepperTask2(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(500));
	PRINT("[stepper2] Task started - running continuously FORWARD\r\n");

	for (;;) {
		stepper2_run(STEPPER_FORWARD, 200, STEPPER_SPEED_MED);

		if (xEventGroupGetBits(emergencyGroup) & OBSTACLE_ACTIVE_BIT) {
			PRINT("[stepper2] Emergency detected - waiting for path clear\r\n");
			xEventGroupWaitBits(emergencyGroup, PATH_CLEAR_BIT, pdTRUE, pdFALSE,
			portMAX_DELAY);
			PRINT("[stepper2] Resuming\r\n");
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
	MX_TIM3_Init(STEPPER_SPEED_MED);
	MX_FDCAN1_Init();

	FDCAN_FilterTypeDef sFilterConfig = { 0 };
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
		Error_Handler();
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
		Error_Handler();
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		Error_Handler();

	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 6, 0);

	printMutex = xSemaphoreCreateMutex();
	i2cMutex = xSemaphoreCreateMutex();
	emergencyGroup = xEventGroupCreate();
	canRxQueue = xQueueCreate(10, sizeof(CAN_MsgTypeDef));

	printf("[MCU2] System init OK\r\n");

	xTaskCreate(gpioTask, "gpioTask", 128, NULL, 1, NULL);
	xTaskCreate(bmi160Task, "bmiTask", 384, NULL, 3, &bmiTaskHandle);

	// --- COMMENT THESE OUT FOR NOW TO PREVENT INTERRUPT CRASHES WHILE TESTING ---
//	xTaskCreate(stepperTask, "stepper1", 256, NULL, 2, &stepTaskHandle);
//	xTaskCreate(stepperTask2, "stepper2", 256, NULL, 2, &stepTaskHandle2);
//	xTaskCreate(canRxTask, "canRxTask", 512, NULL, 4, NULL);
//	xTaskCreate(emergencyHandlerTask, "emergTask", 256, NULL, 5, NULL);
//	xTaskCreate(telemetryTask, "telemTask", 256, NULL, 2, NULL);

	/* Initialise flash storage for checkpoints */
	flash_storage_init();

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
// TIM3 INIT
// ============================================================
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

	// PC13 — LED
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// PA1=DIR1, PA2=STEP1, PA8=DIR2, PA9=STEP2 — GPIO outputs
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PB2 — BMI160 INT1
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	// PB5=SDA, PB6=SCL — I2C1 AF4
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
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
