/**
 * @file    wifi_ap_ws.h
 * @brief   WiFi SoftAP + HTTP/WebSocket server for phone joystick control.
 *
 *  - AP SSID: "RobotControl"  Password: "robot1234"  IP: 192.168.4.1
 *  - HTTP GET /   → serves the embedded joystick HTML page
 *  - WS  GET /ws  → bidirectional JSON channel
 *
 *  Phone → ESP32 (commands):
 *    {"cmd":"move",     "dir":0-3, "steps":N, "speed":N}
 *    {"cmd":"stop"}
 *    {"cmd":"save_cp",  "id":0-15}
 *    {"cmd":"set_mode", "mode":0-2}
 *
 *  ESP32 → Phone (telemetry, decoded from incoming CAN frames):
 *    {"type":"telemetry", "steps_l":N, "steps_r":N, "heading":N, "mode":N, "last_cp":N}
 *    {"type":"imu",       "ax":N, "ay":N, "gz":N}
 *    {"type":"ack",       "cp_id":N, "result":0|1}
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise WiFi SoftAP and start the HTTP/WebSocket server.
 *
 *  Prerequisites (must be done before calling this):
 *    - nvs_flash_init()
 *    - esp_netif_init()
 *    - esp_event_loop_create_default()
 *    - twai_start()   (TWAI driver ready to accept twai_transmit calls)
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_ap_ws_init(void);

/**
 * @brief Broadcast a JSON string to all connected WebSocket clients.
 *        Thread-safe; can be called from any FreeRTOS task.
 *
 * @param json_str  NULL-terminated JSON string (max ~256 bytes recommended)
 * @return ESP_OK, ESP_FAIL if server not started, ESP_ERR_NO_MEM on alloc fail
 */
esp_err_t wifi_ap_ws_broadcast(const char *json_str);

/**
 * @brief Update the table list stored in RAM and NVS.
 *        Called from BLE command handler when cmd=set_tables arrives.
 *
 * @param tables_array  JSON array string, e.g. ["T1","T2","T3"]
 */
void wifi_ap_ws_set_tables(const char *tables_array);

/**
 * @brief Mark all pending orders for a table as delivered (status=1).
 *        Called from BLE cmd handler when cmd=order_delivered arrives.
 */
void wifi_ap_ws_mark_delivered(const char *table);

/**
 * @brief Set or clear the emergency-stop flag.
 *        While active, move/stop WS commands are silently dropped.
 *        Called from canRxTask on 0x301 (obstacle) / 0x303 (path clear).
 */
void wifi_ap_ws_set_emergency(bool active);

#ifdef __cplusplus
}
#endif
