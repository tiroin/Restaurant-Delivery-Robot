# u8g2 (ESP-IDF Fork)

> 本仓库基于 [olikraus/u8g2](https://github.com/olikraus/u8g2) fork，
> 针对 ESP-IDF 工程做了较大目录重组与文件删减，并增加了 ESP32 硬件端口与组件化构建能力。

## 1. 项目定位

这是一个用于 ESP-IDF 的 `u8g2` 组件版本，目标是：

- 保留 `u8g2` 核心绘图能力；
- 适配 ESP-IDF 5.x 组件构建；
- 提供可直接用于 ESP32 的 I2C/SPI 硬件后端；
- 通过 Kconfig 选择显示驱动家族，按需编译，减少体积与编译时间。

## 2. 与上游 u8g2 的主要差异

相较上游仓库，本 fork 主要做了以下改动：

1. **目录结构重组**：
	- 核心源码放在 `src/core/`
	- 显示驱动放在 `src/driver/`
	- ESP32 端口放在 `src/port/`
2. **文件删减**：删除与当前 ESP-IDF 组件形态无关内容，保留核心可用子集。
3. **新增 ESP32 硬件端口**：
	- I2C: `u8x8_byte_esp32_hw_i2c`, `u8x8_gpio_and_delay_esp32_i2c`
	- SPI: `u8x8_byte_esp32_hw_spi`, `u8x8_gpio_and_delay_esp32_spi`
4. **新增组件配置能力**：通过 Kconfig 选择显示驱动家族，并在 CMake 中只编译对应 `u8x8_d_*.c`。

## 3. 目录说明

- `include/`：对外头文件（`u8g2.h`, `u8x8.h` 以及 ESP32 端口头）
- `src/core/`：u8g2 核心逻辑
- `src/driver/`：控制器驱动实现（`u8x8_d_*.c`）
- `src/port/`：ESP-IDF 硬件适配层（I2C/SPI）
- `CMakeLists.txt`：ESP-IDF 组件构建规则
- `Kconfig.projbuild`：驱动家族选择配置
- `idf_component.yml`：组件元数据

## 4. 版本与依赖

- ESP-IDF: `>= 5.0`
- 组件元数据见 `idf_component.yml`

## 5. Kconfig 驱动选择（新增）

进入 `menuconfig`：

- `u8g2 Configuration`
- `u8g2 display driver family`

默认是 **`all drivers`**（兼容旧行为）。

当你选择某一驱动家族（如 `ssd1306`）时，仅会编译：

- 公共驱动层：`u8g2_d_*.c`
- 该家族驱动：`u8x8_d_ssd1306*.c`

## 6. ESP32 I2C 使用示例（最小流程）

1. 初始化 `u8g2_esp32_i2c_ctx_t`，填写 `i2c_port/sda/scl/addr`。
2. 调用 `u8g2_esp32_i2c_set_default_context()`。
3. 选择对应 setup（例如 `u8g2_Setup_ssd1306_i2c_128x64_noname_f`）。
4. 调用 `u8x8_SetI2CAddress()`（注意传入 8-bit 地址，即 7-bit 左移 1）。
5. `u8g2_InitDisplay()` + `u8g2_SetPowerSave(..., 0)` 后开始绘制。

可参考工程示例调用方式：
- `main/app-main.cpp`

## 7. ESP32 SPI 使用说明

SPI 流程与 I2C 类似：

1. 初始化 `u8g2_esp32_spi_ctx_t`（host/sclk/mosi/cs/dc/reset 等）；
2. 调用 `u8g2_esp32_spi_set_default_context()`；
3. 在 setup 中使用 `u8x8_byte_esp32_hw_spi` 与 `u8x8_gpio_and_delay_esp32_spi`。

## 8. 迁移提示（从上游到本 fork）

- 请优先包含本组件导出的头文件；
- 若你直接依赖上游目录布局，请按本仓库 `src/core|driver|port` 重新适配路径；
- 驱动裁剪后，setup 函数需与所选驱动家族匹配，否则会链接失败。

## 9. 许可证与致谢

- 本项目继承上游 `u8g2` 的许可条款（见 `LICENSE`）。
- 感谢 `olikraus/u8g2` 社区的长期维护与贡献。

