# ESP32-S3 Wireless Serial Bridge (ESP-NOW)

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.0+-blue.svg)](https://github.com/espressif/esp-idf)

Proyek ini mengimplementasikan jembatan serial nirkabel menggunakan dua ESP32-S3 yang berkomunikasi melalui ESP-NOW. Sistem ini memungkinkan transfer data serial secara wireless antara PC dan mikrokontroler dengan dukungan penuh untuk kontrol aliran data (DTR/RTS) dan konfigurasi line coding.

## 🎯 Fitur Utama

- ✅ **Transfer Data Bi-directional** - Data dapat mengalir dua arah secara simultan
- ✅ **USB CDC Device & USB Host** - Mendukung koneksi USB di kedua sisi
- ✅ **Line Coding Support** - Baudrate, parity, stop bits, data bits
- ✅ **Control Lines** - DTR/RTS signal forwarding
- ✅ **Checksum Validation** - Integritas data terjamin
- ✅ **LED Indicators** - Visual feedback untuk RX/TX/DTR/RTS
- ✅ **Auto-reconnection** - Otomatis reconnect jika device terputus
- ✅ **Heartbeat Mechanism** - Monitoring koneksi real-time

## 📋 Arsitektur Sistem
```
Arduino IDE / PC
       ↓ (USB)
ESP32-S3 #1 (Sender)
  [USB CDC Device]
       ↓ (ESP-NOW)
ESP32-S3 #2 (Receiver)
   [USB Host]
       ↓ (USB)
USB-to-TTL Converter
       ↓ (UART)
Mikrokontroler Target
```

## 🛠️ Hardware Requirements

### ESP32-S3 Sender (USB CDC Device)
- **Board**: ESP32-S3 DevKit atau sejenisnya
- **USB**: Native USB (bukan via USB-to-Serial chip)
- **LEDs** (opsional):
  - GPIO 21: RX LED
  - GPIO 47: TX LED
  - GPIO 35: DTR LED
  - GPIO 36: RTS LED

### ESP32-S3 Receiver (USB Host)
- **Board**: ESP32-S3 DevKit dengan USB Host support
- **USB**: USB OTG port
- **USB-to-TTL Converter**: FTDI, CP2102, CH340, atau sejenisnya

### Additional Components
- Kabel USB Type-C (2 buah)
- USB-to-TTL Converter
- Mikrokontroler target (Arduino, STM32, dll)
- Breadboard dan kabel jumper (opsional)

## 📦 Software Requirements

- **ESP-IDF**: v5.0 atau lebih baru
- **Python**: 3.8+ (untuk ESP-IDF tools)
- **Git**: Untuk clone repository

## 🚀 Installation

### 1. Setup ESP-IDF
```bash
# Clone ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout release/v5.0  # atau versi lebih baru

# Install dependencies
./install.sh  # Linux/macOS
# atau
install.bat   # Windows

# Setup environment
. ./export.sh  # Linux/macOS
# atau
export.bat     # Windows
```

### 2. Clone Project
```bash
git clone https://github.com/username/esp32-wireless-serial-bridge.git
cd esp32-wireless-serial-bridge
```

### 3. Struktur Project
```
esp32-wireless-serial-bridge/
├── sender/
│   ├── main/
│   │   ├── usb_to_esp_now.c
│   │   ├── data_packet.h
│   │   └── CMakeLists.txt
│   ├── CMakeLists.txt
│   └── sdkconfig
├── receiver/
│   ├── main/
│   │   ├── esp_now_to_usb.cpp
│   │   ├── data_packet.h
│   │   └── CMakeLists.txt
│   ├── CMakeLists.txt
│   └── sdkconfig
└── README.md
```

## ⚙️ Configuration

### 1. Dapatkan MAC Address

Sebelum memulai, Anda perlu mendapatkan MAC address dari kedua ESP32-S3:
```bash
# Flash kode sederhana untuk membaca MAC
idf.py flash monitor
# Catat MAC address yang muncul di log
```

### 2. Konfigurasi Sender

Edit file `sender/main/usb_to_esp_now.c`:
```c
// Line 27: Ganti dengan MAC address RECEIVER
static uint8_t receiver_mac[ESP_NOW_ETH_ALEN] = {0x98, 0xA3, 0x16, 0xE5, 0x78, 0x68};
```

### 3. Konfigurasi Receiver

Edit file `receiver/main/esp_now_to_usb.cpp`:
```cpp
// Line 48: Ganti dengan MAC address SENDER
static uint8_t sender_mac[ESP_NOW_ETH_ALEN] = {0xC0, 0x4E, 0x30, 0x0A, 0xBD, 0x40};
```

### 4. Konfigurasi LED (Opsional)

Edit file `sender/main/usb_to_esp_now.c` untuk menyesuaikan GPIO LED:
```c
// Line 61-64
#define LED_RX_GPIO  GPIO_NUM_21
#define LED_TX_GPIO  GPIO_NUM_47
#define LED_DTR_GPIO GPIO_NUM_35
#define LED_RTS_GPIO GPIO_NUM_36
```

### 5. WiFi Channel

Kedua device **HARUS** menggunakan WiFi channel yang sama. Default: **Channel 1**

Untuk mengubah channel, edit di kedua file:
```c
// Sender: line 221
ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

// Receiver: line 246
ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
```

## 🔨 Build & Flash

### Build Sender
```bash
cd sender
idf.py set-target esp32s3
idf.py menuconfig  # Opsional: konfigurasi tambahan
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Build Receiver
```bash
cd receiver
idf.py set-target esp32s3
idf.py menuconfig  # Opsional: konfigurasi tambahan
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

**Note**: Ganti `/dev/ttyUSB0` dan `/dev/ttyUSB1` dengan port serial yang sesuai di sistem Anda.

## 📝 Usage

### 1. Koneksi Hardware

**Sender Side:**
```
PC/Arduino IDE --[USB]--> ESP32-S3 Sender
```

**Receiver Side:**
```
ESP32-S3 Receiver --[USB]--> USB-to-TTL --[UART]--> Mikrokontroler Target
```

### 2. Testing dengan Serial Monitor

1. Upload sketch sederhana ke mikrokontroler target:
```cpp
void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("Echo: ");
    Serial.println(c);
  }
}
```

2. Buka Arduino IDE Serial Monitor
3. Connect ke ESP32-S3 Sender
4. Set baudrate 115200
5. Kirim data - harusnya terlihat echo dari mikrokontroler target

### 3. LED Indicators

- **RX LED** (GPIO 21): Berkedip saat menerima data dari PC
- **TX LED** (GPIO 47): Berkedip saat mengirim data via ESP-NOW
- **DTR LED** (GPIO 35): Berkedip saat DTR signal berubah
- **RTS LED** (GPIO 36): Berkedip saat RTS signal berubah

## 🔍 Monitoring & Debugging

### Monitor Logs
```bash
# Sender
idf.py -p /dev/ttyUSB0 monitor

# Receiver
idf.py -p /dev/ttyUSB1 monitor
```

### Log Output Example

**Sender:**
```
I (1234) CDC_SENDER: CDC RX: 10 bytes queued (seq: 42)
I (1235) CDC_SENDER: Sent packet seq=42, type=0, len=10
I (6234) CDC_SENDER: Heartbeat sent
```

**Receiver:**
```
I (1234) ESPNOW_RECEIVER: ESP-NOW RX: seq=42, type=0, len=10
I (1235) ESPNOW_RECEIVER: Sent 10 bytes to USB device
I (5234) ESPNOW_RECEIVER: Heartbeat received
```

### Common Issues

#### 1. ESP-NOW Packet Send Failed

**Problem**: `ESP-NOW send failed` di log

**Solution**:
- Pastikan kedua device menggunakan WiFi channel yang sama
- Verifikasi MAC address sudah benar
- Check jarak antar device (max ~100m line-of-sight)

#### 2. USB Device Not Detected

**Problem**: `No VCP device found` di receiver

**Solution**:
- Check koneksi USB
- Pastikan USB-to-TTL converter didukung (FTDI/CP210x/CH34x)
- Coba reconnect USB device

#### 3. Data Corruption

**Problem**: Data yang diterima tidak sesuai

**Solution**:
- Check `Checksum validation failed` di log
- Pastikan tidak ada interference WiFi
- Reduce baudrate jika terlalu tinggi

#### 4. LED Not Working

**Problem**: LED tidak menyala

**Solution**:
- Verifikasi GPIO pin configuration
- Check LED polarity (pastikan anode ke GPIO, cathode ke GND via resistor)
- Pastikan `LED_BLINK_DURATION_MS` tidak terlalu kecil

## 📊 Performance

### Latency
- **Typical**: 10-30ms (one-way)
- **Maximum**: 50ms (under heavy load)

### Throughput
- **ESP-NOW Max Packet**: 250 bytes
- **Effective Throughput**: ~100-200 KB/s (tergantung baudrate dan traffic)

### Range
- **Line-of-Sight**: ~100m
- **Indoor**: ~30-50m (tergantung obstacles)

## 🔧 Advanced Configuration

### Custom Packet Queue Size

Edit `data_packet.h`:
```c
#define MAX_DATA_SIZE 250  // ESP-NOW max payload
```

Edit `usb_to_esp_now.c`:
```c
#define QUEUE_SIZE 20  // Sender queue size
```

Edit `esp_now_to_usb.cpp`:
```cpp
#define ESPNOW_TO_USB_QUEUE_SIZE 100  // Receiver queue size
```

### Baudrate Support

System mendukung baudrate standar:
- 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600

### Custom WiFi Channel

Pilih channel yang least crowded di environment Anda (1-13):
```c
ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));
```

## 📚 API Reference

### Data Packet Structure
```c
typedef struct {
    uint32_t time_code;           // Timestamp (microseconds)
    uint16_t sequence_number;     // Packet sequence number
    uint8_t  packet_type;         // Type: DATA/LINE_CODING/CONTROL_LINE/HEARTBEAT
    uint16_t data_len;            // Actual data length
    uint8_t  data[MAX_DATA_SIZE]; // Serial data
    bool     dtr_state;           // DTR state
    bool     rts_state;           // RTS state
    line_coding_t line_coding;    // Line coding parameters
    uint16_t checksum;            // Checksum for validation
} data_packet_t;
```

### Packet Types
```c
typedef enum {
    PACKET_TYPE_DATA = 0,          // Serial data
    PACKET_TYPE_LINE_CODING = 1,   // Baudrate/parity/stop bits change
    PACKET_TYPE_CONTROL_LINE = 2,  // DTR/RTS change
    PACKET_TYPE_HEARTBEAT = 3,     // Keep-alive packet
    PACKET_TYPE_ACK = 4            // Acknowledgment (not used)
} packet_type_t;
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the project
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Espressif Systems untuk ESP-IDF framework
- TinyUSB untuk USB CDC implementation
- ESP-NOW protocol documentation

## 📧 Contact

Project Link: https://github.com/muhamad-bekti-wibowo/ESP32-S3-Wireless-Serial-Bridge-ESP-NOW

---

## 🔖 Version History

- **v1.0.0** (2024-10-19)
  - Initial release
  - Basic bi-directional communication
  - LED indicators
  - Line coding support
  - Control line forwarding

---

🧑‍💻 Dibuat oleh Muhamad Bekti Wibowo

📅 Tahun 2025
