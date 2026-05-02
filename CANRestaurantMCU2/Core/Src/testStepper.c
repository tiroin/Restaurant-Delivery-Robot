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
#include "FreeRTOS.h"
#include "task.h"
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

#define DIR_FORWARD     GPIO_PIN_SET
#define DIR_BACKWARD    GPIO_PIN_RESET

/* ── Speed presets (timer period in µs) ─────────────────────── */
#define STEPPER_SPEED_SLOW  2000
#define STEPPER_SPEED_MED   1000
#define STEPPER_SPEED_FAST   500

/* ── Handles ────────────────────────────────────────────────── */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

static SemaphoreHandle_t printMutex   = NULL;
static TaskHandle_t      stepTask1    = NULL;
static TaskHandle_t      stepTask2    = NULL;

/* ── Step counters (written by task, decremented by ISR) ─────── */
static volatile uint32_t stepRemaining  = 0;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ── Stepper 1 — TIM2 ── */
    if (htim->Instance == TIM2)
    {
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
    if (htim->Instance == TIM3)
    {
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
static void MX_TIM2_Init(uint32_t period)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 249;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = period - 1;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
}

static void stepper1_run(GPIO_PinState dir, uint32_t steps, uint32_t speed)
{
    if (steps == 0) return;
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir);
    stepRemaining = steps;
    MX_TIM2_Init(speed);
    HAL_TIM_Base_Start_IT(&htim2);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* block until done */
}

static void stepper1_stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    stepRemaining = 0;
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════
 * STEPPER 2 DRIVER  (TIM3)
 * ═══════════════════════════════════════════════════════════════ */
static void MX_TIM3_Init(uint32_t period)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 249;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = period - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
}

static void stepper2_run(GPIO_PinState dir, uint32_t steps, uint32_t speed)
{
    if (steps == 0) return;
    HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, dir);
    stepRemaining2 = steps;
    MX_TIM3_Init(speed);
    HAL_TIM_Base_Start_IT(&htim3);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* block until done */
}

static void stepper2_stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    stepRemaining2 = 0;
    HAL_GPIO_WritePin(STEP2_PORT, STEP2_PIN, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 1 — STEPPER 1
 * Runs forward continuously, 200 steps per batch.
 * Change DIR_FORWARD to DIR_BACKWARD to reverse.
 * Change STEPPER_SPEED_MED to SLOW or FAST to adjust speed.
 * ═══════════════════════════════════════════════════════════════ */
static void stepperTask1(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));   /* let system settle */
    PRINT("[stepper1] Task started\r\n");

    for (;;)
    {
        stepper1_run(DIR_FORWARD, 200, STEPPER_SPEED_MED);
        /* add vTaskDelay here if you want a pause between batches */
    }
}

/* ═══════════════════════════════════════════════════════════════
 * TASK 2 — STEPPER 2
 * Exact mirror of task 1.
 * ═══════════════════════════════════════════════════════════════ */
static void stepperTask2(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    PRINT("[stepper2] Task started\r\n");

    for (;;)
    {
        stepper2_run(DIR_FORWARD, 200, STEPPER_SPEED_MED);
    }
}

/* ═══════════════════════════════════════════════════════════════
 * BLINK TASK — PC13 heartbeat so you know the scheduler is alive
 * ═══════════════════════════════════════════════════════════════ */
static void blinkTask(void *arg)
{
    (void)arg;
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ═══════════════════════════════════════════════════════════════
 * GPIO INIT
 * PA1=DIR1  PA2=STEP1  PA8=DIR2  PA9=STEP2  PC13=LED
 * ═══════════════════════════════════════════════════════════════ */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* PA1, PA2, PA8, PA9 — stepper outputs */
    HAL_GPIO_WritePin(GPIOA,
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9,
        GPIO_PIN_RESET);
    g.Pin   = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* PC13 — LED */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

/* ═══════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();   /* keep your existing SystemClock_Config */
    MX_GPIO_Init();
    MX_TIM2_Init(STEPPER_SPEED_MED);
    MX_TIM3_Init(STEPPER_SPEED_MED);

    HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    printMutex = xSemaphoreCreateMutex();

    printf("[MCU2] Stepper-only init OK\r\n");

    xTaskCreate(blinkTask,    "blink",    128, NULL, 1, NULL);
    xTaskCreate(stepperTask1, "stepper1", 256, NULL, 2, &stepTask1);
    xTaskCreate(stepperTask2, "stepper2", 256, NULL, 2, &stepTask2);

    vTaskStartScheduler();
    while (1) {}
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
