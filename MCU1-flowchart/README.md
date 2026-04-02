# 🔐 Smart Door Lock with ESP RainMaker

Hệ thống khóa cửa thông minh sử dụng **ESP32-C3** với nhiều lớp bảo mật: RFID, mật khẩu, điều khiển từ xa qua ESP RainMaker, và giám sát thống kê chi tiết.

## ✨ Tính năng chính

### 🔑 Xác thực đa lớp
- **RFID RC522**: Quét thẻ MIFARE 13.56MHz
- **Keypad 4x4**: Nhập mật khẩu 6 số (mỗi thẻ có password riêng)
- **Admin Card**: Thẻ đặc biệt để vào menu quản trị
- **ESP RainMaker**: Điều khiển từ xa qua app iOS/Android

### 📱 Quản lý thông qua RainMaker App
- **Mở cửa từ xa**: Tự động đóng sau 3 giây
- **Nút Exit vật lý (GPIO 18)**: Người trong nhà bấm để ra ngoài, cửa tự đóng sau 3s
- **Thống kê chi tiết**: 
  - Tổng số lần mở cửa
  - RFID thành công/thất bại
  - Cảnh báo xâm nhập
  - Số lần mở từ app
  - Người dùng truy cập nhiều nhất
- **Push Notifications**: Cảnh báo khi có người lạ

### 🔒 Bảo mật nâng cao
- **Buzzer Alert**: Kêu 3 lần + thông báo app khi:
  - Mật khẩu sai
  - Thẻ chưa đăng ký
- **NVS Storage**: Lưu dữ liệu thẻ/mật khẩu bền vững
- **Admin Menu**: Chỉ truy cập bằng thẻ admin

### 🖥️ Giao diện OLED
- Hiển thị trạng thái real-time
- Hướng dẫn sử dụng
- Menu quản trị trực quan

### 🌐 Hoạt động Offline
- WiFi kết nối trong background task riêng
- RFID hoạt động ngay cả khi không có mạng
- Tự động đồng bộ khi có kết nối

---

## 🛠️ Phần cứng yêu cầu

### Board chính
- **ESP32-C3-DevKitC** (hoặc tương tự)

### Các module cần thiết

| Module | GPIO | Giao thức | Mục đích |
|--------|------|-----------|----------|
| **MFRC522 RFID** | MISO=2, MOSI=6, CLK=7, CS=10, RST=3 | SPI | Đọc thẻ MIFARE |
| **PCF8574 Keypad** | SDA=4, SCL=5 | I2C (0x20) | Nhập mật khẩu 4x4 |
| **SSD1306 OLED** | SDA=4, SCL=5 | I2C (0x3C) | Hiển thị trạng thái |
| **Relay Module** | GPIO 8 | Digital Out | Điều khiển khóa cửa |
| **Buzzer/LED** | GPIO 1 | PWM (LEDC) | Cảnh báo âm thanh/ánh sáng |
| **Exit Button** | GPIO 18 | Digital In | Nút thoát cho người trong nhà |

### Sơ đồ kết nối

```
ESP32-C3 DevKitC
├─ SPI Bus (RFID RC522)
│  ├─ MISO → GPIO 2
│  ├─ MOSI → GPIO 6
│  ├─ SCK  → GPIO 7
│  ├─ CS   → GPIO 10
│  └─ RST  → GPIO 3
│
├─ I2C Bus (Keypad + OLED)
│  ├─ SDA  → GPIO 4
│  └─ SCL  → GPIO 5
│
├─ Relay Control
│  └─ GPIO 8 → Relay IN (HIGH = mở cửa)
│
├─ Exit Button (Inside)
│  └─ GPIO 18 → Push Button (Active LOW)
│
└─ Security Alert
   └─ GPIO 1 → Buzzer + LED (PWM 2.5kHz)
```

---

## 📦 Cài đặt & Build

### 1. Cài đặt ESP-IDF v5.4+
```bash
# Windows
F:\.espressif\Espressif\frameworks\esp-idf-v5.4.2\export.bat

# Linux/Mac
. $HOME/esp/esp-idf/export.sh
```

### 2. Clone & Build
```bash
cd esp-rainmaker/examples/switch
idf.py set-target esp32c3
idf.py build
```

### 3. Flash firmware
```bash
idf.py -p COM3 flash monitor
```

### 4. Provisioning (lần đầu)
- Quét QR code trên serial monitor
- Mở app **ESP RainMaker** (iOS/Android)
- Thêm device và cấu hình WiFi

---

## 🎯 Hướng dẫn sử dụng

### Lần đầu khởi động
1. **Đăng ký Admin Card**:
   - Quét thẻ đầu tiên → tự động trở thành Admin
   - Mật khẩu mặc định: `000000`

2. **Thêm người dùng mới**:
   - Quét Admin Card → Nhập password admin
   - Chọn `3 - Them the moi`
   - Quét thẻ mới → Đặt tên → Đặt mật khẩu 6 số

### Mở cửa thường ngày
- **Cách 1**: Quét thẻ RFID → Nhập password → ✅ Cửa mở 3 giây
- **Cách 2**: Mở app RainMaker → Bấm nút **Switch** → ✅ Cửa mở 3 giây
- **Cách 3**: Bấm **nút EXIT vật lý** (GPIO 18) → ✅ Cửa mở 3 giây (cho người trong nhà ra ngoài)

### Menu Admin (chỉ Admin Card)
```
========== MENU ADMIN ==========
1 - Xem mat khau
2 - Doi mat khau
3 - Them the moi
4 - Xoa the
0 - Thoat
```

### Thống kê trên App
Vào device **"Thống Kê"** trong RainMaker app để xem:
- 📊 Tổng số lần mở
- ✅ RFID thành công
- ❌ RFID thất bại
- ⚠️ Cảnh báo xâm nhập
- 📱 Mở từ app
- 👤 Người dùng nhiều nhất

---

## ⚙️ Cấu hình nâng cao

### Thay đổi GPIO (menuconfig)
```bash
idf.py menuconfig
→ Example Configuration
  → Output GPIO (Relay): 8
  → Board Button GPIO: 9
  → Exit Button GPIO: 18
```

### Thay đổi thời gian mở cửa
**File: `main/app_main.c`** (RFID + App)
```c
#define DOOR_OPEN_TIME 3000  // 3 giây (đổi thành 5000 = 5 giây)
```

**File: `main/app_driver.c`** (Exit Button)
```c
#define AUTO_CLOSE_DELAY_MS 3000  // 3 giây
```

### Thay đổi tần số Buzzer/LED
```c
.freq_hz = 2500,  // 2.5kHz (cao hơn = to hơn)
```

---

## 🔧 Troubleshooting

### ❌ RFID không đọc được thẻ
- Kiểm tra kết nối SPI (MISO, MOSI, CLK, CS, RST)
- Đưa thẻ gần module (< 5cm)
- Kiểm tra nguồn 3.3V của RC522

### ❌ Keypad không nhận phím
- Kiểm tra địa chỉ I2C: `i2cdetect` (phải thấy 0x20)
- Kiểm tra pull-up resistor trên SDA/SCL

### ❌ Relay không bật
- Kiểm tra GPIO 8 bằng multimeter
- Đảm bảo relay module cần 5V (nối Vcc vào 5V, không phải 3.3V)
- Relay active LOW thì đổi `RELAY_ON_LEVEL = 0`

### ❌ WiFi không kết nối
- Xóa NVS: `idf.py erase-flash`
- Provisioning lại qua QR code
- Kiểm tra WiFi 2.4GHz (ESP32 không hỗ trợ 5GHz)

### ❌ Stack Overflow
- Tăng stack size của task trong code:
```c
xTaskCreate(task_func, "name", 4096, ...);  // Tăng từ 2048 → 4096
```

---

## 📚 Cấu trúc code

```
main/
├── app_main.c           # Logic chính: RFID, RainMaker, thống kê
├── app_driver.c         # Điều khiển GPIO relay
├── app_priv.h           # Định nghĩa chung
├── MFRC522.c/h          # Driver RFID RC522
├── keypad_i2c.c/h       # Driver PCF8574 keypad
├── card_storage.c/h     # Lưu/đọc thẻ từ NVS Flash
├── oled_i2c.c/h         # Driver SSD1306 OLED
└── Kconfig.projbuild    # Cấu hình menuconfig
```

### Các hàm chính
- `write_cb()`: Xử lý lệnh từ RainMaker app
- `open_door()`: Mở relay + reset RC522
- `update_stats_to_rainmaker()`: Đồng bộ thống kê
- `alert_intruder()`: Buzzer + push notification
- `admin_menu()`: Menu quản trị thẻ/password

---

## 📊 Tài nguyên hệ thống

### Bộ nhớ ESP32-C3
| Loại | Tổng | Đã dùng | Còn lại | Mục đích |
|------|------|---------|---------|----------|
| **Flash** | 4 MB | ~1.8 MB | ~2.2 MB | Firmware + RainMaker libs |
| **SRAM** | 400 KB | ~180 KB | ~220 KB | Runtime + WiFi + Tasks |
| **NVS Flash** | 24 KB | ~8 KB | ~16 KB | Cards (50 thẻ) + Config |

### Phân bổ GPIO (ESP32-C3)
| GPIO | Chức năng | Giao thức | Trạng thái | Ghi chú |
|------|-----------|-----------|------------|---------|
| **0** | (Reserved) | - | ⚠️ Boot Mode | Không dùng |
| **1** | Buzzer + LED | PWM (LEDC) | ✅ Output | 2.5kHz, 10-bit |
| **2** | RFID MISO | SPI | ✅ Input | RC522 data out |
| **3** | RFID RST | GPIO | ✅ Output | Reset RC522 |
| **4** | I2C SDA | I2C | ✅ Bi-dir | Keypad + OLED |
| **5** | I2C SCL | I2C | ✅ Output | Clock 100kHz |
| **6** | RFID MOSI | SPI | ✅ Output | RC522 data in |
| **7** | RFID CLK | SPI | ✅ Output | SPI clock |
| **8** | Relay Control | GPIO | ✅ Output | HIGH = mở cửa |
| **9** | Boot Button | GPIO | ✅ Input | Factory reset |
| **10** | RFID CS | SPI | ✅ Output | Chip select |
| **12-17** | - | - | 🔓 Free | Dự phòng mở rộng |
| **18** | Exit Button | GPIO | ✅ Input | Người trong nhà ra |
| **19** | USB | - | ⚠️ Reserved | JTAG/Serial |
| **20,21** | UART | - | ⚠️ Reserved | Console log |

### Dung lượng NVS (Non-Volatile Storage)
| Namespace | Key | Kích thước | Số lượng | Tổng |
|-----------|-----|------------|----------|------|
| **cards** | card_0 ~ card_49 | ~160 bytes/thẻ | 50 thẻ | ~8 KB |
| **rmaker** | node_id, certificates | ~4 KB | - | 4 KB |
| **wifi** | ssid, password | ~100 bytes | - | <1 KB |
| **stats** | counters | 40 bytes | - | <1 KB |
| **Total** | - | - | - | **~14 KB / 24 KB** |

**Cấu trúc 1 thẻ trong NVS:**
```c
typedef struct {
    uint8_t uid[10];        // 10 bytes - UID thẻ RFID
    char name[32];          // 32 bytes - Tên chủ thẻ
    char password[7];       // 7 bytes - Mật khẩu 6 số + null
    uint8_t uid_length;     // 1 byte - Độ dài UID
    uint32_t last_access;   // 4 bytes - Timestamp truy cập cuối
} AllowedCard;              // = 54 bytes + padding → ~160 bytes
```

### FreeRTOS Tasks
| Task | Stack Size | Priority | CPU Core | Mục đích |
|------|------------|----------|----------|----------|
| **main** | 4096 B | 1 | Core 0 | RFID scan loop |
| **wifi_connect** | 4096 B | 5 | Core 0 | WiFi background |
| **relay_close** | 4096 B | 5 | Core 0 | Auto-close relay |
| **mqtt_task** | 6144 B | 5 | Core 0 | RainMaker MQTT |
| **idle** | 1024 B | 0 | Core 0 | FreeRTOS idle |

**Tổng RAM động:** ~180 KB / 400 KB

### Partition Table (4MB Flash)
```
# Name          Type    SubType   Offset    Size
nvs             data    nvs       0x9000    24K
otadata         data    ota       0xF000    8K
phy_init        data    phy       0x11000   4K
ota_0           app     ota_0     0x20000   1536K
ota_1           app     ota_1     0x1A0000  1536K
fctry           app     factory   0x320000  1024K
storage         data    spiffs    0x420000  768K
```

**Lợi ích OTA 2 partition:**
- ✅ Cập nhật firmware từ xa an toàn
- ✅ Rollback tự động nếu update lỗi
- ✅ Không làm gián đoạn hoạt động

---

## 🚀 Nâng cấp tương lai

### Đã triển khai ✅
- [x] RFID + Password authentication
- [x] Admin menu quản lý thẻ
- [x] ESP RainMaker remote control
- [x] Exit button cho người trong nhà (GPIO 18)
- [x] Offline-first operation
- [x] Auto-close relay (3s) với reset RC522 antenna
- [x] Security alerts (buzzer + push notification)
- [x] Statistics dashboard

### Ý tưởng mở rộng 💡
- [ ] **Battery Backup**: Module UPS 5V + 18650 cells
- [ ] **Fingerprint**: Cảm biến vân tay FPM10A
- [ ] **Face Recognition**: ESP32-S3 + OV2640 camera
- [ ] **Telegram Bot**: Thông báo + điều khiển qua Telegram
- [ ] **Time-based Access**: Giới hạn thời gian truy cập
- [ ] **Guest Cards**: Thẻ tạm thời tự hủy
- [ ] **Tamper Detection**: Cảm biến chống phá khóa
- [ ] **Voice Control**: Tích hợp Google Assistant/Alexa

---

## 📄 License

This example is in the Public Domain (or CC0 licensed).

---

## 🤝 Đóng góp

Mọi đóng góp, báo lỗi, và ý tưởng cải tiến đều được chào đón!

---

## 📞 Hỗ trợ

- ESP RainMaker Docs: https://rainmaker.espressif.com/docs/
- ESP-IDF Docs: https://docs.espressif.com/projects/esp-idf/
- Forum: https://esp32.com/

---

**Made with ❤️ using ESP32-C3 & ESP RainMaker**
