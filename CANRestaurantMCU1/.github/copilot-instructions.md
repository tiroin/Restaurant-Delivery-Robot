# ESP RainMaker Switch Example - AI Coding Guide

## Project Overview
This is an ESP32 IoT firmware example built on ESP-IDF that connects a physical switch device to the ESP RainMaker cloud platform. The device can be controlled locally (via hardware button) and remotely (via RainMaker mobile app).

## Architecture

### Core Components
- **app_main.c**: RainMaker node initialization, event handling, device registration, and cloud connectivity
- **app_driver.c**: Hardware abstraction layer for GPIO, buttons (iot_button), and LED indicators (led_indicator)
- **app_priv.h**: Shared definitions between app layers
- **Managed components**: ESP IDF Component Manager handles dependencies in `main/idf_component.yml`

### Key Dependencies (idf_component.yml)
- `espressif/esp_rainmaker`: Cloud connectivity and device management
- `espressif/button`: Hardware button event handling
- `espressif/led_indicator`: RGB/WS2812 LED control abstraction
- `espressif/rmaker_app_*`: Common app utilities (network, reset, insights) located at `../../common/`

### Event-Driven Architecture
The application uses ESP-IDF's event loop system extensively:
- **RMAKER_EVENT**: Node lifecycle (init, claim, local control)
- **RMAKER_COMMON_EVENT**: Cloud connection states (MQTT, WiFi reset, factory reset)
- **APP_NETWORK_EVENT**: Provisioning events (QR display, timeouts)
- **RMAKER_OTA_EVENT**: Firmware update lifecycle

All events are registered in `app_main()` and handled by the centralized `event_handler()` function.

## Build System

### ESP-IDF CMake Structure
- Root `CMakeLists.txt` sets `RMAKER_PATH` environment variable and includes IDF project
- `main/CMakeLists.txt` registers component sources with `idf_component_register()`
- Build artifacts live in `build/` directory

### Configuration Hierarchy
1. `sdkconfig.defaults`: Base configuration for all targets
2. `sdkconfig.defaults.esp32*`: Target-specific overrides (ESP32, ESP32-C3, ESP32-S2, etc.)
3. `Kconfig.projbuild`: Custom project options under "Example Configuration" menu

Key config patterns in `sdkconfig.defaults`:
```
CONFIG_PARTITION_TABLE_CUSTOM=y  # Custom partition layout
CONFIG_ESP_RMAKER_LOCAL_CTRL_AUTO_ENABLE=y  # Secure local control
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y  # OTA safety
CONFIG_BT_NIMBLE_ENABLED=y  # BLE provisioning (not ESP32-S2)
```

### Standard Build Commands
```bash
idf.py build                    # Compile project
idf.py -p PORT flash           # Flash to device
idf.py -p PORT monitor         # Serial monitor
idf.py -p PORT flash monitor   # Flash and monitor
idf.py menuconfig              # Configure via GUI
idf.py fullclean               # Clean everything
```

## Hardware Abstraction Patterns

### GPIO Configuration
Hardware pins configured via Kconfig (`Kconfig.projbuild`):
- `CONFIG_EXAMPLE_BOARD_BUTTON_GPIO`: Boot button (default varies by chip)
- `CONFIG_EXAMPLE_OUTPUT_GPIO`: Relay/output control (default 19)
- `CONFIG_WS2812_LED_GPIO` or `CONFIG_RGB_LED_*_GPIO`: LED pins

### LED Type Abstraction
Three mutually exclusive LED types via `LED_TYPE` Kconfig choice:
- `CONFIG_LED_TYPE_RGB`: 3-pin RGB with separate R/G/B GPIOs
- `CONFIG_LED_TYPE_WS2812`: Addressable LED strip via RMT peripheral
- `CONFIG_LED_TYPE_NONE`: Headless operation

Driver code uses `#ifdef CONFIG_LED_TYPE_*` to conditionally compile hardware-specific init code.

### Button Event Handling
Uses `iot_button` component for debounced GPIO buttons:
```c
button_handle_t btn_handle = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn_handle);
iot_button_register_cb(btn_handle, BUTTON_SINGLE_CLICK, NULL, push_btn_cb, NULL);
```

Reset functionality layered on same button via `app_reset_button_register()`:
- 3 seconds → WiFi reset
- 10 seconds → Factory reset

## RainMaker Device Model

### Device Creation Pattern
```c
esp_rmaker_device_t *device = esp_rmaker_device_create("Name", ESP_RMAKER_DEVICE_SWITCH, NULL);
esp_rmaker_device_add_cb(device, write_cb, NULL);  // Handle cloud commands
esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(...));
esp_rmaker_device_add_param(device, esp_rmaker_power_param_create(...));
esp_rmaker_node_add_device(node, device);
```

### Parameter Update Pattern
- **Local updates**: `esp_rmaker_param_update_and_report()` - Report state change to cloud
- **Cloud commands**: Arrive via `write_cb()` callback, update hardware, then `esp_rmaker_param_update()`
- **Notifications**: `esp_rmaker_param_update_and_notify()` triggers push notifications

### Initialization Order (Critical!)
```c
app_network_init();           // BEFORE esp_rmaker_node_init()
esp_rmaker_node_init(...);
// ... register devices and params ...
esp_rmaker_start();
app_network_start();          // AFTER esp_rmaker_start()
```

## Code Conventions

### Naming
- `app_*`: Application-specific functions
- `CONFIG_EXAMPLE_*`: Project-specific Kconfig options
- `DEFAULT_*`: Compile-time constants (e.g., `DEFAULT_POWER`)
- Global device handles: `switch_device` (extern in header)

### Error Handling
- Use `ESP_ERROR_CHECK()` for initialization failures
- Critical failures abort with 5-second delay: `vTaskDelay(5000/portTICK_PERIOD_MS); abort();`
- Non-critical errors return early with `ESP_ERR_*` codes

### Logging
- Tag pattern: `static const char *TAG = "component_name";`
- Use ESP-IDF log macros: `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE`
- Log format specifiers: `%"PRIi32"` for int32_t, `%s` for strings

## Multi-Target Support
Project supports ESP32, ESP32-S2, ESP32-C2, ESP32-C3, ESP32-C6, ESP32-H2 via:
- Target-specific `sdkconfig.defaults.esp32*` files
- Conditional Kconfig defaults (e.g., button GPIO varies by chip)
- `#ifdef IDF_TARGET_ESP32*` in code (use sparingly - prefer Kconfig)

## Testing & Debug
- Enable `CONFIG_EXAMPLE_ENABLE_TEST_NOTIFICATIONS` to test push notifications
- Monitor logs for event lifecycle: provisioning → claimed → MQTT connected
- Factory reset via button hold for fresh provisioning cycle
