/**
 * @file    ble_robot.h
 * @brief   BLE GATT peripheral for robot control (replaces wifi_ap_ws.h)
 *
 *  Runs a Nordic UART Service (NUS) GATT peripheral:
 *    NUS Service:  6e400001-b5a3-f393-e0a9-e50e24dcca9e
 *    RX (CMD):     6e400002-b5a3-f393-e0a9-e50e24dcca9e  Write/WriteNoRsp (phone→ESP32)
 *    TX (TEL):     6e400003-b5a3-f393-e0a9-e50e24dcca9e  Notify (ESP32→phone)
 *
 *  Commands from phone are JSON strings identical to the old WebSocket protocol.
 *  Telemetry to phone is JSON strings identical to the old WebSocket protocol.
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise NimBLE host and start GATT server + advertising.
 *         Call once from app_main before creating tasks.
 *         NVS must already be initialised before calling this.
 */
esp_err_t ble_robot_init(void);

/**
 * @brief  Send a JSON telemetry string to the connected phone via BLE notify.
 *         Returns ESP_FAIL if no client is connected or notification fails.
 *
 * @param  json_str  Null-terminated JSON string (must fit in one BLE packet, ≤512 bytes)
 */
esp_err_t ble_robot_notify(const char *json_str);

/**
 * @brief  Returns true if a phone is currently connected over BLE.
 */
bool ble_robot_connected(void);

#ifdef __cplusplus
}
#endif
