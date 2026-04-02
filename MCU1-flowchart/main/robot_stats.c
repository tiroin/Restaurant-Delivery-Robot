/**
 * @file    robot_stats.c
 * @brief   Robot runtime statistics — Implementation
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "robot_stats.h"

static const char *TAG = "RobotStats";

/* Internal state */
static robot_stats_t  s_stats;
static SemaphoreHandle_t s_mutex = NULL;

/* ─────────────────────────────────────────────────────────── */

void robot_stats_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.current_state      = ROBOT_STATE_IDLE;
    s_stats.current_checkpoint = CHECKPOINT_ID_HOME;
    s_stats.battery_percent    = 100;

    ESP_LOGI(TAG, "Stats initialized");
}

/* ─────────────────────────────────────────────────────────── */

void robot_stats_snapshot(robot_stats_t *out)
{
    if (!out) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_stats;
    xSemaphoreGive(s_mutex);
}

/* ─────────────────────────────────────────────────────────── */

void robot_stats_set_state(robot_state_t state)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stats.current_state = state;
    xSemaphoreGive(s_mutex);
}

void robot_stats_set_checkpoint(uint8_t checkpoint_id)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stats.current_checkpoint = checkpoint_id;
    xSemaphoreGive(s_mutex);
}

void robot_stats_set_battery(uint8_t percent)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stats.battery_percent = percent;
    xSemaphoreGive(s_mutex);
}

void robot_stats_set_error(const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    strncpy(s_stats.last_error, buf, sizeof(s_stats.last_error) - 1);
    s_stats.last_error[sizeof(s_stats.last_error) - 1] = '\0';
    xSemaphoreGive(s_mutex);

    ESP_LOGW(TAG, "Error: %s", buf);
}

/* ─────────────────────────────────────────────────────────── */

void robot_stats_inc_orders(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stats.total_orders_served++;
    xSemaphoreGive(s_mutex);
}

void robot_stats_inc_trips(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stats.total_trips++;
    xSemaphoreGive(s_mutex);
}

void robot_stats_inc_emergency(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stats.emergency_stops++;
    xSemaphoreGive(s_mutex);
}

void robot_stats_inc_obstacle(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stats.obstacle_detections++;
    xSemaphoreGive(s_mutex);
}

/* ─────────────────────────────────────────────────────────── */

const char *robot_state_to_str(robot_state_t state)
{
    switch (state) {
        case ROBOT_STATE_IDLE:           return "IDLE";
        case ROBOT_STATE_GOING_KITCHEN:  return "GOING_KITCHEN";
        case ROBOT_STATE_WAITING_LOAD:   return "WAITING_LOAD";
        case ROBOT_STATE_GOING_TABLE:    return "GOING_TABLE";
        case ROBOT_STATE_WAITING_UNLOAD: return "WAITING_UNLOAD";
        case ROBOT_STATE_RETURNING_HOME: return "RETURNING_HOME";
        case ROBOT_STATE_EMERGENCY:      return "EMERGENCY";
        case ROBOT_STATE_ERROR:          return "ERROR";
        case ROBOT_STATE_CHARGING:       return "CHARGING";
        default:                         return "UNKNOWN";
    }
}
