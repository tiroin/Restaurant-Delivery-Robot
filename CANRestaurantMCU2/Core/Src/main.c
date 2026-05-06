/*
 * main.c — MCU2: BMI160 IMU + FDCAN + Dual Stepper  (FreeRTOS)
 *
 *  Hardware:
 *    I2C1   PB5(SDA) PB6(SCL)   → BMI160 @ 0x68
 *    PB2    EXTI rising          → BMI160 INT1 (DRDY)
 *    FDCAN1 PB10(TX) PB12(RX)   @ 1 Mbps
 *    TIM2   PA1(DIR1) PA2(STEP1) → Stepper 1 (left)
 *    TIM3   PA8(DIR2) PA9(STEP2) → Stepper 2 (right)
 *    PC13   LED heartbeat
 *
 *  CAN TX:  0x211 accel (ax*100 ay*100 az*100 BE16)
 *           0x213 gyro  (gx*100 gy*100 gz*100 BE16)
 *  CAN RX:  0x110 MANUAL_MOVE (dir 0-3, steps BE16, speed BE16)
 *           0x111 SAVE_CP     (checkpoint_id)
 *           0x105 HEARTBEAT
 */

#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ── BMI160 ──────────────────────────────────────────────────── */
#define BMI160_ADDR             (0x68 << 1)
#define BMI160_REG_CHIP_ID      0x00
#define BMI160_REG_DATA_START   0x0C
#define BMI160_REG_INT_EN_1     0x51
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_MAP_1    0x56
#define BMI160_REG_CMD          0x7E
#define BMI160_CMD_SOFT_RESET   0xB6
#define BMI160_CMD_ACC_NORMAL   0x11
#define BMI160_CMD_GYR_NORMAL   0x15
#define BMI160_CHIP_ID_VAL      0xD1

/* ── Stepper pins ────────────────────────────────────────────── */
#define STEP_PORT    GPIOA
#define STEP_PIN     GPIO_PIN_2
#define DIR_PORT     GPIOA
#define DIR_PIN      GPIO_PIN_1
#define STEP2_PORT   GPIOA
#define STEP2_PIN    GPIO_PIN_9
#define DIR2_PORT    GPIOA
#define DIR2_PIN     GPIO_PIN_8
#define DIR1_FORWARD   GPIO_PIN_SET
#define DIR1_BACKWARD  GPIO_PIN_RESET
#define DIR2_FORWARD   GPIO_PIN_RESET
#define DIR2_BACKWARD  GPIO_PIN_SET
#define STEPPER_SPEED_CRAWL  4000U   /* 125 steps/s — heavy load */

/* ── CAN IDs ─────────────────────────────────────────────────── */
#define CAN_ID_IMU_ACCEL        0x211U  /* MCU2 → MCU1 */
#define CAN_ID_IMU_GYRO         0x213U  /* MCU2 → MCU1 */
#define CAN_ID_MANUAL_MOVE      0x110U  /* MCU1 → MCU2: dir,steps,speed */
#define CAN_ID_SAVE_CP          0x111U  /* MCU1 → MCU2: checkpoint_id */
#define CAN_ID_HEARTBEAT_MCU1   0x105U  /* MCU1 → MCU2 */
#define CAN_ID_CP_SAVED_ACK     0x212U  /* MCU2 → MCU1: data[0]=cp_id, data[1]=0 OK */
#define CAN_ID_ODOMETRY         0x214U  /* MCU2 → MCU1: dir,s1_hi,s1_lo,s2_hi,s2_lo,tk_hi,tk_lo */
#define CAN_ID_EMERGENCY_STOP   0x302U  /* MCU3 → MCU2: front-sensor emergency */
#define CAN_ID_PATH_CLEAR       0x303U  /* MCU3 → MCU2: front clear, resume saved leg */
#define STEP_COUNT_THRESHOLD    200U    /* steps >= this in NavCmd_t → closed-loop step-count mode */

/* ── Types ───────────────────────────────────────────────────── */
typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} CAN_Msg_t;

typedef struct {
	uint8_t dir;
	uint16_t steps;
	uint16_t speed;
} NavCmd_t;

/* ── Peripheral handles ──────────────────────────────────────── */
I2C_HandleTypeDef hi2c1;
FDCAN_HandleTypeDef hfdcan1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* ── RTOS handles ────────────────────────────────────────────── */
static SemaphoreHandle_t printMutex = NULL;
static SemaphoreHandle_t i2cMutex = NULL;
static SemaphoreHandle_t canTxMutex = NULL;
static QueueHandle_t canRxQueue = NULL;
static QueueHandle_t stepQueue1 = NULL; /* mailbox depth=1 */
static QueueHandle_t stepQueue2 = NULL; /* mailbox depth=1 */
static TaskHandle_t bmiTaskHandle = NULL;
static TaskHandle_t imuTxHandle = NULL;
static TaskHandle_t stepTask1 = NULL;
static TaskHandle_t stepTask2 = NULL;

/* ── IMU globals (bmi160Task → imuTxTask) ────────────────────── */
static volatile int16_t g_ax = 0, g_ay = 0, g_az = 0;
static volatile int16_t g_gx = 0, g_gy = 0, g_gz = 0;
static volatile uint32_t g_imuCount = 0;

/* ── Stepper step counters (legacy, unused) ─────────────────── */
static volatile uint32_t stepRemaining = 0;
static volatile uint32_t stepRemaining2 = 0;
/* ── Hardware step counters & closed-loop targets ────────────── */
static volatile uint32_t motor1_steps = 0;  /* full steps since last start (ISR-written) */
static volatile uint32_t motor2_steps = 0;
static volatile uint32_t stepTarget1 = 0;   /* 0 = unlimited; ISR self-stops when reached */
static volatile uint32_t stepTarget2 = 0;

/* ── Emergency-stop state (set by 0x302, cleared by 0x303) ─────────────
 * On 0x302 we snapshot the in-flight leg's remaining steps as the FIRST
 * entry in g_emrgQ, then every step-count MANUAL_MOVE that arrives during
 * the emergency is appended to g_emrgQ. On 0x303 we replay the queue in
 * FIFO order: pop[0] -> push to stepper queues; each leg-completion in
 * stepperTask1 (around send_odometry) pops the next.                  */
#define EMRG_QDEPTH 16U
static volatile bool g_emergencyStop = false;
static NavCmd_t g_emrgQ[EMRG_QDEPTH];
static volatile uint8_t g_emrgQHead = 0;   /* next pop  */
static volatile uint8_t g_emrgQTail = 0;   /* next push */
static volatile uint8_t g_emrgQCount = 0;
/* Last MANUAL_MOVE issued (used to build the in-flight remainder leg). */
static volatile uint8_t  g_curDir   = 0;
static volatile uint16_t g_curSpeed = 0;

/* ── Forward declarations ────────────────────────────────────── */
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM2_Init(uint32_t period);
static void MX_TIM3_Init(uint32_t period);

/* ── Print helper ────────────────────────────────────────────── */
#define PRINT(...) do { \
    if (printMutex) xSemaphoreTake(printMutex, portMAX_DELAY); \
    printf(__VA_ARGS__); \
    if (printMutex) xSemaphoreGive(printMutex); \
} while (0)

/* Verbose debug-only print: enabled for emergency-stop tracing.
 * Set DBG_VERBOSE to 0 once the issue is resolved. */
#define DBG_VERBOSE 1
#define DPRINT(...) do { if (DBG_VERBOSE) PRINT(__VA_ARGS__); } while (0)

/* ═══════════════════════════════════════════════════════════════
 * FDCAN RX FIFO0 CALLBACK  (ISR)
 * ═══════════════════════════════════════════════════════════════ */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
		return;
	CAN_Msg_t msg = { 0 };
	FDCAN_RxHeaderTypeDef rxHdr;
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHdr, msg.data)
			== HAL_OK) {
		msg.id = rxHdr.Identifier;
		msg.len = (uint8_t) rxHdr.DataLength;
		BaseType_t woken = pdFALSE;
		if (canRxQueue)
			xQueueSendFromISR(canRxQueue, &msg, &woken);
		portYIELD_FROM_ISR(woken);
	}
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
	if (hfdcan->Instance->PSR & FDCAN_PSR_BO) {
		HAL_FDCAN_Stop(hfdcan);
		HAL_FDCAN_Start(hfdcan);
		HAL_FDCAN_ActivateNotification(hfdcan,
				FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF
						| FDCAN_IT_ARB_PROTOCOL_ERROR
						| FDCAN_IT_DATA_PROTOCOL_ERROR, 0);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TIM2 / TIM3 PERIOD ELAPSED ISR
 * Free-running: each ISR toggles the STEP pin for one half-period.
 * The task starts/stops the timer; no step counting in ISR.
 * ═══════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		static uint8_t ps1 = 0;
		if (ps1 == 0) {
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			ps1 = 1;
		} else {
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			ps1 = 0;
			motor1_steps++;  /* count on falling edge = 1 full step complete */
			if (stepTarget1 > 0 && motor1_steps >= stepTarget1) {
				/* Self-stop: disable timer from within ISR */
				__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
				__HAL_TIM_DISABLE(&htim2);
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
				if (stepTask1) {
					BaseType_t xHPW = pdFALSE;
					vTaskNotifyGiveFromISR(stepTask1, &xHPW);
					portYIELD_FROM_ISR(xHPW);
				}
			}
		}
	}
	if (htim->Instance == TIM3) {
		static uint8_t ps2 = 0;
		if (ps2 == 0) {
			HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_SET);
			ps2 = 1;
		} else {
			HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_RESET);
			ps2 = 0;
			motor2_steps++;
			if (stepTarget2 > 0 && motor2_steps >= stepTarget2) {
				__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
				__HAL_TIM_DISABLE(&htim3);
				HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_RESET);
				/* stepTask2 notified; no odometry send (stepTask1 handles that) */
				if (stepTask2) {
					BaseType_t xHPW = pdFALSE;
					vTaskNotifyGiveFromISR(stepTask2, &xHPW);
					portYIELD_FROM_ISR(xHPW);
				}
			}
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * EXTI PB2 — BMI160 INT1 (DRDY)
 * ═══════════════════════════════════════════════════════════════ */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_2 && bmiTaskHandle) {
		BaseType_t w = pdFALSE;
		vTaskNotifyGiveFromISR(bmiTaskHandle, &w);
		portYIELD_FROM_ISR(w);
	}
}

/* ═══════════════════════════════════════════════════════════════
 * CAN TX HELPER
 * ═══════════════════════════════════════════════════════════════ */
static HAL_StatusTypeDef can_tx(uint32_t id, const uint8_t *data, uint8_t len) {
	static const uint32_t dlc_table[] = {
	FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
	FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
	FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8, };
	FDCAN_TxHeaderTypeDef txHdr = { 0 };
	txHdr.Identifier = id;
	txHdr.IdType = FDCAN_STANDARD_ID;
	txHdr.TxFrameType = FDCAN_DATA_FRAME;
	txHdr.DataLength = (len <= 8) ? dlc_table[len] : FDCAN_DLC_BYTES_8;
	txHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHdr.BitRateSwitch = FDCAN_BRS_OFF;
	txHdr.FDFormat = FDCAN_CLASSIC_CAN;
	txHdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	if (canTxMutex)
		xSemaphoreTake(canTxMutex, portMAX_DELAY);
	HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHdr,
			(uint8_t*) data);
	if (canTxMutex)
		xSemaphoreGive(canTxMutex);
	return ret;
}

/* ═══════════════════════════════════════════════════════════════
 * STEPPER DRIVERS
 * ═══════════════════════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════════════════════
 * STEPPER HELPERS — continuous mode
 * The timer runs freely once started; task calls _stop to halt it.
 * Speed is fixed at crawl, init is done once at boot.
 * ═══════════════════════════════════════════════════════════════ */
/* target=0 → unlimited (continuous); target>0 → ISR self-stops after target full steps */
static void stepper1_start(GPIO_PinState dir, uint32_t target) {
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir);
	motor1_steps = 0;
	stepTarget1 = target;   /* set BEFORE enabling ISR */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim2);
	PRINT("[M1] START dir=%u tgt=%lu emrg=%u\r\n", (unsigned)dir, target, (unsigned)g_emergencyStop);
}
static void stepper1_stop(void) {
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
	PRINT("[M1] STOP m1=%lu tgt=%lu\r\n", motor1_steps, stepTarget1);
}
static void stepper2_start(GPIO_PinState dir, uint32_t target) {
	HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, dir);
	motor2_steps = 0;
	stepTarget2 = target;
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim3);
	PRINT("[M2] START dir=%u tgt=%lu emrg=%u\r\n", (unsigned)dir, target, (unsigned)g_emergencyStop);
}
static void stepper2_stop(void) {
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_RESET);
	PRINT("[M2] STOP m2=%lu tgt=%lu\r\n", motor2_steps, stepTarget2);
}

/* ── Emergency replay FIFO helpers ──────────────────────────────
 * All callers run in task context; canRxTask is highest-prio of the
 * three writers (canRx > stepperTask1/2). Brief critical sections
 * via taskENTER_CRITICAL keep head/tail/count consistent. */
static bool emrgQ_push(NavCmd_t cmd) {
	bool ok = false;
	taskENTER_CRITICAL();
	if (g_emrgQCount < EMRG_QDEPTH) {
		g_emrgQ[g_emrgQTail] = cmd;
		g_emrgQTail = (uint8_t)((g_emrgQTail + 1U) % EMRG_QDEPTH);
		g_emrgQCount++;
		ok = true;
	}
	taskEXIT_CRITICAL();
	return ok;
}
/* Push to front (head) — used by 0x302 so the current leg's remainder
 * is replayed FIRST, even if earlier queued legs are already waiting. */
static bool emrgQ_push_front(NavCmd_t cmd) {
	bool ok = false;
	taskENTER_CRITICAL();
	if (g_emrgQCount < EMRG_QDEPTH) {
		g_emrgQHead = (uint8_t)((g_emrgQHead + EMRG_QDEPTH - 1U) % EMRG_QDEPTH);
		g_emrgQ[g_emrgQHead] = cmd;
		g_emrgQCount++;
		ok = true;
	}
	taskEXIT_CRITICAL();
	return ok;
}
static bool emrgQ_pop(NavCmd_t *out) {
	bool ok = false;
	taskENTER_CRITICAL();
	if (g_emrgQCount > 0) {
		*out = g_emrgQ[g_emrgQHead];
		g_emrgQHead = (uint8_t)((g_emrgQHead + 1U) % EMRG_QDEPTH);
		g_emrgQCount--;
		ok = true;
	}
	taskEXIT_CRITICAL();
	return ok;
}
static void emrgQ_clear(void) __attribute__((unused));
static void emrgQ_clear(void) {
	taskENTER_CRITICAL();
	g_emrgQHead = g_emrgQTail = g_emrgQCount = 0;
	taskEXIT_CRITICAL();
}
/* Pop the next pending leg (if any) and dispatch to stepper queues.
 * Also updates g_curDir/g_curSpeed so a subsequent 0x302 (obstacle
 * detected mid-replay) can correctly snapshot the in-flight remainder.
 * No-op while emergency is active (caller must clear flag first).     */
static void emrgQ_advance(const char *why) {
	if (g_emergencyStop)
		return;
	NavCmd_t cmd;
	if (!emrgQ_pop(&cmd))
		return;
	/* Track so 0x302 knows the current direction/speed during replay. */
	g_curDir   = cmd.dir;
	g_curSpeed = cmd.speed;
	xQueueOverwrite(stepQueue1, &cmd);
	xQueueOverwrite(stepQueue2, &cmd);
	PRINT("[EMRG] %s -> replay dir=%u steps=%u (q=%u left)\r\n",
			why ? why : "advance", cmd.dir, cmd.steps, g_emrgQCount);
}

/* Send odometry CAN frame (called by stepperTask1 only). */
static void send_odometry(uint8_t dir) {
	uint16_t s1 = (uint16_t)(motor1_steps > 0xFFFF ? 0xFFFF : motor1_steps);
	uint16_t s2 = (uint16_t)(motor2_steps > 0xFFFF ? 0xFFFF : motor2_steps);
	uint16_t tk = (uint16_t)(HAL_GetTick() & 0xFFFF);
	uint8_t buf[7] = {
		dir,
		(uint8_t)(s1 >> 8), (uint8_t)(s1 & 0xFF),
		(uint8_t)(s2 >> 8), (uint8_t)(s2 & 0xFF),
		(uint8_t)(tk >> 8), (uint8_t)(tk & 0xFF)
	};
	DPRINT("[ODO] dir=%u s1=%u s2=%u t=%lu\r\n", dir, s1, s2, HAL_GetTick());
	can_tx(CAN_ID_ODOMETRY, buf, 7);
}

/* ═══════════════════════════════════════════════════════════════
 * TASK — LED heartbeat (PC13, 500 ms)
 * ═══════════════════════════════════════════════════════════════ */
static void ledTask(void *arg) {
	(void) arg;
	for (;;) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK — BMI160 (DRDY-driven via EXTI PB2)
 * ═══════════════════════════════════════════════════════════════ */
static void bmi160Task(void *arg) {
	(void) arg;
	vTaskDelay(pdMS_TO_TICKS(200));
	uint8_t id = 0, cmd;

	xSemaphoreTake(i2cMutex, portMAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, BMI160_ADDR, BMI160_REG_CHIP_ID, 1, &id, 1, 100);
	if (id != BMI160_CHIP_ID_VAL) {
		PRINT("[BMI160] Bad CHIP_ID 0x%02X\r\n", id);
		xSemaphoreGive(i2cMutex);
		vTaskDelete(NULL);
	}
	cmd = BMI160_CMD_SOFT_RESET;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
	xSemaphoreGive(i2cMutex);
	vTaskDelay(pdMS_TO_TICKS(15));

	xSemaphoreTake(i2cMutex, portMAX_DELAY);
	cmd = BMI160_CMD_ACC_NORMAL;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
	xSemaphoreGive(i2cMutex);
	vTaskDelay(pdMS_TO_TICKS(15));

	xSemaphoreTake(i2cMutex, portMAX_DELAY);
	cmd = BMI160_CMD_GYR_NORMAL;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
	cmd = 0x0A;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_INT_OUT_CTRL, 1, &cmd, 1,
			100);
	cmd = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_INT_MAP_1, 1, &cmd, 1,
			100);
	cmd = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_INT_EN_1, 1, &cmd, 1,
			100);
	xSemaphoreGive(i2cMutex);
	PRINT("[BMI160] Init OK — DRDY on PB2\r\n");

	uint8_t raw[12], printDiv = 0;
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		xSemaphoreTake(i2cMutex, portMAX_DELAY);
		HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, BMI160_ADDR,
		BMI160_REG_DATA_START, 1, raw, 12, 100);
		xSemaphoreGive(i2cMutex);
		if (ret != HAL_OK)
			continue;

		g_gx = (int16_t) ((raw[1] << 8) | raw[0]);
		g_gy = (int16_t) ((raw[3] << 8) | raw[2]);
		g_gz = (int16_t) ((raw[5] << 8) | raw[4]);
		g_ax = (int16_t) ((raw[7] << 8) | raw[6]);
		g_ay = (int16_t) ((raw[9] << 8) | raw[8]);
		g_az = (int16_t) ((raw[11] << 8) | raw[10]);
		g_imuCount++;

		if (++printDiv >= 50) {
			printDiv = 0;
			DPRINT("[IMU] A=%.2f,%.2f,%.2fg  G=%.1f,%.1f,%.1f dps\r\n",
					g_ax / 16384.0f, g_ay / 16384.0f, g_az / 16384.0f,
					g_gx / 16.4f, g_gy / 16.4f, g_gz / 16.4f);
			if (imuTxHandle)
				xTaskNotifyGive(imuTxHandle); /* signal imuTxTask every 50 samples */
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK — IMU CAN TX  (notification-driven by bmi160Task every 50 samples)
 *   0x211: ax*100 ay*100 az*100 BE16
 *   0x213: gx*100 gy*100 gz*100 BE16
 * ═══════════════════════════════════════════════════════════════ */
static void imuTxTask(void *arg) {
	(void) arg;
	uint32_t txCount = 0, txFail = 0;
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* block until bmi160Task signals */

		int16_t ax100 = (int16_t) ((int32_t) g_ax * 100 / 16384);
		int16_t ay100 = (int16_t) ((int32_t) g_ay * 100 / 16384);
		int16_t az100 = (int16_t) ((int32_t) g_az * 100 / 16384);
		/* gyro: 16.4 LSB/dps (±2000 dps default). raw*100/164 = dps×10  (0.1°/s units) */
		int16_t gx100 = (int16_t) ((int32_t) g_gx * 100 / 164);
		int16_t gy100 = (int16_t) ((int32_t) g_gy * 100 / 164);
		int16_t gz100 = (int16_t) ((int32_t) g_gz * 100 / 164);

		uint8_t abuf[6] = { (uint8_t) (ax100 >> 8), (uint8_t) ax100,
				(uint8_t) (ay100 >> 8), (uint8_t) ay100, (uint8_t) (az100 >> 8),
				(uint8_t) az100, };
		uint8_t gbuf[6] = { (uint8_t) (gx100 >> 8), (uint8_t) gx100,
				(uint8_t) (gy100 >> 8), (uint8_t) gy100, (uint8_t) (gz100 >> 8),
				(uint8_t) gz100, };

		HAL_StatusTypeDef ra = can_tx(CAN_ID_IMU_ACCEL, abuf, 6);
		HAL_StatusTypeDef rg = can_tx(CAN_ID_IMU_GYRO, gbuf, 6);
		if (ra == HAL_OK && rg == HAL_OK) {
			txCount++;
			if ((txCount % 10) == 0)
				DPRINT("[CAN-TX] #%lu  A=%d,%d,%d  G=%d,%d,%d\r\n", txCount,
						ax100, ay100, az100, gx100, gy100, gz100);
		} else {
			txFail++;
			PRINT("[CAN-TX] FAIL #%lu  PSR=0x%08lX\r\n", txFail,
					hfdcan1.Instance->PSR);
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK — Stepper 1  (left, CAN-driven)
 *   dir=0 FWD  → DIR1_BACKWARD  (physical forward)
 *   dir=1 BWD  → DIR1_FORWARD
 *   dir=2 LEFT → DIR1_BACKWARD  (left motor backward = pivot left)
 *   dir=3 RIGHT→ DIR1_FORWARD
 * ═══════════════════════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════════════════════
 * TASK — Stepper 1  (left, CAN-driven) — continuous mode
 *   Motor runs freely while move commands arrive within 300 ms.
 *   Stops on explicit stop (steps==0) or 300 ms queue timeout.
 * ═══════════════════════════════════════════════════════════════ */
static void stepperTask1(void *arg) {
	(void) arg;
	PRINT("[stepper1] Task started\r\n");
	bool running = false;
	bool stepMode = false;  /* true = waiting for ISR step-target notification */
	uint8_t lastDir = 0;
	uint32_t scStartTick = 0;
	uint8_t hbDiv = 0;
	for (;;) {
		/* ── Step-count mode: poll for ISR notification or stop command ── */
		if (stepMode) {
			uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
			/* Heartbeat every ~200ms */
			if (++hbDiv >= 10) {
				hbDiv = 0;
				PRINT("[S1] HB m1=%lu tgt=%lu emrg=%u q=%u t=%lu\r\n",
					motor1_steps, stepTarget1,
					(unsigned)g_emergencyStop, g_emrgQCount,
					HAL_GetTick() - scStartTick);
			}
			if (!notified) {
				/* Not done yet — check for explicit stop command */
				NavCmd_t ovr;
				if (xQueueReceive(stepQueue1, &ovr, 0) == pdTRUE && ovr.steps == 0) {
					DPRINT("[S1] STOP@SC m1=%lu tgt=%lu t=%lu\r\n",
						motor1_steps, stepTarget1, HAL_GetTick());
					stepper1_stop(); stepTarget1 = 0;
					running = false; stepMode = false;
					send_odometry(lastDir);
					emrgQ_advance("S1-stop-chain");
				}
				continue;
			}
			/* ISR reached target — timer already disabled by ISR (direct register write).
			   Call HAL stop now so htim->State returns to READY; otherwise the
			   next HAL_TIM_Base_Start_IT silently fails (state stuck at BUSY). */
			stepper1_stop();
			stepTarget1 = 0;
			DPRINT("[S1] ISR-DONE m1=%lu m2=%lu tgt=%lu t=%lu\r\n",
				motor1_steps, motor2_steps, stepTarget1, HAL_GetTick());
			running = false; stepMode = false;
			/* Drain stale keepalives */
			NavCmd_t drain;
			while (xQueueReceive(stepQueue1, &drain, 0) == pdTRUE) {}
			send_odometry(lastDir);
			emrgQ_advance("S1-done-chain");
			continue;
		}
		/* ── Normal mode: wait for queue command ── */
		NavCmd_t cmd;
		BaseType_t got = xQueueReceive(stepQueue1, &cmd,
				running ? pdMS_TO_TICKS(300) : portMAX_DELAY);
		if (got == pdTRUE) {
			DPRINT("[S1] RX dir=%u steps=%u running=%u t=%lu\r\n",
				cmd.dir, cmd.steps, (unsigned)running, HAL_GetTick());
		} else {
			DPRINT("[S1] RX-TIMEOUT 300ms running=%u m1=%lu t=%lu\r\n",
				(unsigned)running, motor1_steps, HAL_GetTick());
		}
		if (got == pdFALSE || cmd.steps == 0) {
			if (running) {
				DPRINT("[S1] TIMEOUT/STOP m1=%lu t=%lu\r\n", motor1_steps, HAL_GetTick());
				stepper1_stop(); stepTarget1 = 0;
				running = false;
				send_odometry(lastDir);
				emrgQ_advance("S1-timeout-chain");
			}
			continue;
		}
		GPIO_PinState dir;
		switch (cmd.dir) {
		case 0: dir = DIR1_BACKWARD; break; /* FWD  */
		case 2: dir = DIR1_BACKWARD; break; /* LEFT */
		default: dir = DIR1_FORWARD; break; /* BWD, RIGHT */
		}
		lastDir = cmd.dir;
		if (!running) {
			uint32_t target = (cmd.steps >= STEP_COUNT_THRESHOLD) ? cmd.steps : 0;
			if (target > 0) {
				DPRINT("[S1] SC-START dir=%u tgt=%lu t=%lu\r\n", cmd.dir, target, HAL_GetTick());
				scStartTick = HAL_GetTick();
				hbDiv = 0;
			}
			stepper1_start(dir, target);
			running = true;
			stepMode = (target > 0);
		} else {
			if (cmd.steps >= STEP_COUNT_THRESHOLD) {
				/* New step-count command while running — restart with new target */
				DPRINT("[S1] SC-RESTART dir=%u tgt=%u t=%lu\r\n", cmd.dir, cmd.steps, HAL_GetTick());
				stepper1_stop(); stepTarget1 = 0;
				stepper1_start(dir, cmd.steps);
				stepMode = true;
				scStartTick = HAL_GetTick();
				hbDiv = 0;
			} else {
				/* Keepalive (steps < threshold) — just update direction */
				HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir);
			}
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK — Stepper 2  (right, mirrored, CAN-driven) — continuous mode
 * ═══════════════════════════════════════════════════════════════ */
static void stepperTask2(void *arg) {
	(void) arg;
	PRINT("[stepper2] Task started\r\n");
	bool running = false;
	bool stepMode = false;
	for (;;) {
		if (stepMode) {
			uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
			if (!notified) {
				NavCmd_t ovr;
				if (xQueueReceive(stepQueue2, &ovr, 0) == pdTRUE && ovr.steps == 0) {
					DPRINT("[S2] STOP@SC m2=%lu tgt=%lu t=%lu\r\n",
						motor2_steps, stepTarget2, HAL_GetTick());
					stepper2_stop(); stepTarget2 = 0;
					running = false; stepMode = false;
				}
				continue;
			}
			/* ISR reached target — normalize HAL state via stepper2_stop. */
			DPRINT("[S2] ISR-DONE m2=%lu tgt=%lu t=%lu\r\n",
				motor2_steps, stepTarget2, HAL_GetTick());
			stepper2_stop();
			stepTarget2 = 0;
			running = false; stepMode = false;
			NavCmd_t drain;
			while (xQueueReceive(stepQueue2, &drain, 0) == pdTRUE) {}
			continue;
		}
		NavCmd_t cmd;
		BaseType_t got = xQueueReceive(stepQueue2, &cmd,
				running ? pdMS_TO_TICKS(300) : portMAX_DELAY);
		if (got == pdTRUE) {
			DPRINT("[S2] RX dir=%u steps=%u speed=%u running=%u t=%lu\r\n",
				cmd.dir, cmd.steps, cmd.speed, (unsigned)running, HAL_GetTick());
		} else {
			DPRINT("[S2] RX-TIMEOUT 300ms running=%u m2=%lu t=%lu\r\n",
				(unsigned)running, motor2_steps, HAL_GetTick());
		}
		if (got == pdFALSE || cmd.steps == 0) {
			if (running) {
				DPRINT("[S2] TIMEOUT/STOP m2=%lu t=%lu\r\n", motor2_steps, HAL_GetTick());
				stepper2_stop(); stepTarget2 = 0; running = false;
			}
			continue;
		}
		GPIO_PinState dir;
		switch (cmd.dir) {
		case 0: dir = DIR2_BACKWARD; break; /* FWD   */
		case 3: dir = DIR2_BACKWARD; break; /* RIGHT */
		default: dir = DIR2_FORWARD; break; /* BWD, LEFT */
		}
		if (!running) {
			uint32_t target = (cmd.steps >= STEP_COUNT_THRESHOLD) ? cmd.steps : 0;
			DPRINT("[S2] START dir=%u tgt=%lu mode=%s t=%lu\r\n",
				cmd.dir, target, target > 0 ? "SC" : "CONT", HAL_GetTick());
			stepper2_start(dir, target);
			running = true;
			stepMode = (target > 0);
		} else {
			if (cmd.steps >= STEP_COUNT_THRESHOLD) {
				DPRINT("[S2] SC-RESTART dir=%u tgt=%u t=%lu\r\n",
					cmd.dir, cmd.steps, HAL_GetTick());
				stepper2_stop(); stepTarget2 = 0;
				stepper2_start(dir, cmd.steps);
				stepMode = true;
			} else {
				DPRINT("[S2] KEEPALIVE dir=%u steps=%u t=%lu\r\n",
					cmd.dir, cmd.steps, HAL_GetTick());
				HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, dir);
			}
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * TASK — CAN RX dispatcher
 *   0x110 MANUAL_MOVE → update nav globals
 *   0x111 SAVE_CP     → log (extend to flash later)
 *   0x105 HEARTBEAT   → log
 * ═══════════════════════════════════════════════════════════════ */
static void canRxTask(void *arg) {
	(void) arg;
	PRINT("[CAN-RX] task started\r\n");
	CAN_Msg_t msg;
	for (;;) {
		xQueueReceive(canRxQueue, &msg, portMAX_DELAY);
		switch (msg.id) {
		case CAN_ID_MANUAL_MOVE: {
			if (msg.len < 5)
				break;
			uint8_t dir = msg.data[0];
			uint16_t steps = (uint16_t) ((msg.data[1] << 8) | msg.data[2]);
			uint16_t speed = (uint16_t) ((msg.data[3] << 8) | msg.data[4]);
			if (speed == 0)
				speed = STEPPER_SPEED_CRAWL;
			if (dir > 3)
				steps = 0;
			/* During emergency: motors stay halted, but every step-count leg
			 * is appended to the replay FIFO so the path is preserved.
			 * Keepalives (steps < threshold) are not legs — ignore them.    */
			if (g_emergencyStop) {
				if (steps >= STEP_COUNT_THRESHOLD) {
					NavCmd_t leg = { dir, steps, speed };
					bool ok = emrgQ_push(leg);
					PRINT("[EMRG] HOLD MANUAL_MOVE dir=%u steps=%u %s (q=%u)\r\n",
							dir, steps,
							ok ? "queued" : "DROPPED-FULL", g_emrgQCount);
				}
				/* Re-assert stop so the 300ms keepalive timeout never restarts. */
				NavCmd_t stop = { 0, 0, 0 };
				xQueueOverwrite(stepQueue1, &stop);
				xQueueOverwrite(stepQueue2, &stop);
				break;
			}
			/* Track latest leg so 0x302 can capture the in-flight remainder. */
			if (steps >= STEP_COUNT_THRESHOLD) {
				g_curDir = dir;
				g_curSpeed = speed;
			}
			NavCmd_t cmd = { dir, steps, speed };
			xQueueOverwrite(stepQueue1, &cmd); /* latest command wins */
			xQueueOverwrite(stepQueue2, &cmd);
			PRINT("[CAN] MOVE dir=%u steps=%u speed=%u curDir=%u curSpd=%u\r\n",
					dir, steps, speed, g_curDir, g_curSpeed);
			break;
		}
		case CAN_ID_EMERGENCY_STOP: {
			/* MCU3 detected obstacle <15cm. Snapshot the in-flight leg's
			 * remaining steps and push it to the FRONT of the replay FIFO
			 * (so it is replayed first, before any already-queued legs).
			 * This also handles a second obstacle arriving during replay:
			 * the partially-done replay leg goes back to head correctly.  */
			uint32_t cur1 = motor1_steps;
			uint32_t tgt1 = stepTarget1;
			uint32_t cur2 = motor2_steps;
			uint32_t tgt2 = stepTarget2;
			uint32_t rem1 = (tgt1 > cur1) ? (tgt1 - cur1) : 0;
			uint32_t rem2 = (tgt2 > cur2) ? (tgt2 - cur2) : 0;
			uint32_t rem = (rem1 > rem2) ? rem1 : rem2; /* dominant axis */
			g_emergencyStop = true;
			NavCmd_t stop = { 0, 0, 0 };
			xQueueOverwrite(stepQueue1, &stop);
			xQueueOverwrite(stepQueue2, &stop);
			if (rem > 0) {
				uint16_t rem16 = (rem > 0xFFFFU) ? 0xFFFFU : (uint16_t) rem;
				uint16_t spd = g_curSpeed ? g_curSpeed : STEPPER_SPEED_CRAWL;
				NavCmd_t leg = { g_curDir, rem16, spd };
				(void) emrgQ_push_front(leg); /* remainder always goes to head */
				PRINT("[EMRG] STOP rem=%u dir=%u (q=%u)\r\n", rem16,
						g_curDir, g_emrgQCount);
			} else {
				PRINT("[EMRG] STOP (no in-flight leg, q=%u)\r\n", g_emrgQCount);
			}
			break;
		}
		case CAN_ID_PATH_CLEAR: {
			/* Path clear — lift the lock and kick off replay of the queued
			 * legs in FIFO order. Subsequent legs are chained automatically
			 * by stepperTask1 calling emrgQ_advance() after each completion.*/
			if (g_emergencyStop) {
				g_emergencyStop = false;
				if (g_emrgQCount > 0) {
					emrgQ_advance("CLEAR");
				} else {
					PRINT("[EMRG] CLEAR (nothing to replay)\r\n");
				}
			}
			break;
		}
		case CAN_ID_SAVE_CP: {
			uint8_t cp_id = msg.data[0];
			// PRINT("[CP] save id=%u\r\n", cp_id);
			/* TODO: persist to flash_storage */
			uint8_t ack[2] = { cp_id, 0 }; /* result 0 = success */
			can_tx(CAN_ID_CP_SAVED_ACK, ack, 2);
			break;
		}
		case CAN_ID_HEARTBEAT_MCU1:
			// PRINT("[HB] cnt=%u\r\n", msg.data[0]);
			break;
		default:
			// PRINT("[CAN-RX] id=0x%03lX len=%u  %02X %02X %02X %02X\r\n", msg.id,
			// 		msg.len, msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
			break;
		}
	}
}

/* ═══════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════ */
int main(void) {
	HAL_Init();
	SystemClock_Config();

	/* ITM/SWO */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	ITM->TCR |= ITM_TCR_ITMENA_Msk;
	ITM->TER |= (1U << 0);

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM2_Init(STEPPER_SPEED_CRAWL);
	MX_TIM3_Init(STEPPER_SPEED_CRAWL);
	MX_FDCAN1_Init();

	HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	/* FDCAN filter — accept all standard IDs into RX FIFO 0 */
	FDCAN_FilterTypeDef f = { 0 };
	f.IdType = FDCAN_STANDARD_ID;
	f.FilterIndex = 0;
	f.FilterType = FDCAN_FILTER_MASK;
	f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	f.FilterID1 = 0x000;
	f.FilterID2 = 0x000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK)
		Error_Handler();
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
		Error_Handler();
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		Error_Handler();

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	HAL_FDCAN_ActivateNotification(&hfdcan1,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF
					| FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
			0);

	printMutex = xSemaphoreCreateMutex();
	i2cMutex = xSemaphoreCreateMutex();
	canTxMutex = xSemaphoreCreateMutex();
	canRxQueue = xQueueCreate(10, sizeof(CAN_Msg_t));
	stepQueue1 = xQueueCreate(1, sizeof(NavCmd_t));
	stepQueue2 = xQueueCreate(1, sizeof(NavCmd_t));

	printf("[MCU2] BMI160 + Stepper init OK\r\n");
	printf("[MCU2] PSR=0x%08lX\r\n", hfdcan1.Instance->PSR);

	xTaskCreate(ledTask, "led", 128, NULL, 1, NULL);
	xTaskCreate(bmi160Task, "bmi", 384, NULL, 10, &bmiTaskHandle);
	xTaskCreate(imuTxTask, "imuTx", 256, NULL, 3, &imuTxHandle);
	xTaskCreate(canRxTask, "canRx", 512, NULL, 54, NULL);
	xTaskCreate(stepperTask1, "stepper1", 256, NULL, 55, &stepTask1);
	xTaskCreate(stepperTask2, "stepper2", 256, NULL, 55, &stepTask2);

	vTaskStartScheduler();
	while (1) {
	}
}

/* ═══════════════════════════════════════════════════════════════
 * PERIPHERAL INIT
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

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef g = { 0 };
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* PA1 PA2 PA8 PA9 — stepper DIR/STEP */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);
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

	/* PB2 — BMI160 INT1  EXTI rising */
	g.Pin = GPIO_PIN_2;
	g.Mode = GPIO_MODE_IT_RISING;
	g.Pull = GPIO_NOPULL;
	g.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &g);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);  /* BMI DRDY: lower than TIM/FDCAN, just wakes a task */
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	/* PB5 PB6 — I2C1 SDA/SCL  AF4 */
	g.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	g.Mode = GPIO_MODE_AF_OD;
	g.Pull = GPIO_PULLUP;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &g);

	/* PB10 — FDCAN1 TX  AF9 */
	g.Pin = GPIO_PIN_10;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &g);

	/* PB12 — FDCAN1 RX  AF9 */
	g.Pin = GPIO_PIN_12;
	g.Pull = GPIO_PULLUP;
	g.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &g);
}

/* ═══════════════════════════════════════════════════════════════
 * CLOCK — CSI → PLL1 → 250 MHz SYSCLK
 * ═══════════════════════════════════════════════════════════════ */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef osc = { 0 };
	RCC_ClkInitTypeDef clk = { 0 };

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	osc.OscillatorType = RCC_OSCILLATORTYPE_CSI;
	osc.CSIState = RCC_CSI_ON;
	osc.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
	osc.PLL.PLLState = RCC_PLL_ON;
	osc.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
	osc.PLL.PLLM = 1;
	osc.PLL.PLLN = 125;
	osc.PLL.PLLP = 2;
	osc.PLL.PLLQ = 2;
	osc.PLL.PLLR = 2;
	osc.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
	osc.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
	osc.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&osc) != HAL_OK)
		Error_Handler();

	clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clk.APB1CLKDivider = RCC_HCLK_DIV1;
	clk.APB2CLKDivider = RCC_HCLK_DIV1;
	clk.APB3CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
		Error_Handler();

	__HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
