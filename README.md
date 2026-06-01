# ESP32-C6 Remote Car

A Wi-Fi controlled miniature car built on the ESP32-C6 microcontroller using ESP-IDF. The car hosts its own Wi-Fi access point and serves a web dashboard for real-time telemetry and motor control. It features automatic obstacle avoidance, IMU-based speed estimation, a TFT status display, and two power-saving sleep modes.

## Features

- **Wi-Fi SoftAP** — creates a standalone hotspot; no router needed
- **Web dashboard** — served at `192.168.4.1`, accessible from any browser
  - Real-time 3D orientation cube driven by accelerometer data
  - Motor control pad (forward / left / right / stop)
  - Distance sensor readout with colour-coded proximity bar
  - G-force metrics per axis
  - Estimated speed gauge
- **Collision avoidance** — forward movement blocked when obstacle < 20 cm; running motors auto-stopped at < 25 cm
- **Speed estimation** — accelerometer integration with EMA low-pass filter, deadband, velocity damping, and zero-velocity update (ZUPT)
- **Sleep modes** — light sleep after 60 s of motor inactivity; deep sleep after a further 60 s or on button press
- **TFT display** — live speed, acceleration, distance, and motor state at 5 Hz
- **PWM LED** — external LED brightness scales linearly with estimated speed (max at 1.5 m/s)
- **Onboard RGB LED** — status indicator: green (active), blue (light sleep), red (deep sleep)

## Hardware

| Component | Description |
|-----------|-------------|
| ESP32-C6 | Main microcontroller (Wi-Fi, I2C, SPI, LEDC, RMT) |
| KS0170 (MPU-6050) | Keyestudio accelerometer module — I2C, used for speed estimation |
| HC-SR04 | Ultrasonic distance sensor |
| ST7735 | 1.8" SPI TFT colour display |
| DC Motors (×2) | Differential drive — left and right independently controlled via ULN2803A |
| ULN2803A | 8-channel Darlington array — translates 3.3 V GPIO signals to motor drive current |
| WS2812B | Onboard addressable RGB LED |
| External LED | PWM-driven brightness indicator |
| Push button | Sleep / wake-up trigger on GPIO 1 |

### Wiring

| Signal | GPIO |
|--------|------|
| I2C SDA (MPU-6050) | 6 |
| I2C SCL (MPU-6050) | 7 |
| TFT MOSI | 19 |
| TFT CLK | 21 |
| TFT CS | 22 |
| TFT DC | 2 |
| TFT RST | 3 |
| TFT Backlight | 15 |
| HC-SR04 TRIG | 4 |
| HC-SR04 ECHO | 5 |
| Sleep button | 1 |
| External LED (PWM) | 0 |
| Motor LEFT | 11 |
| Motor RIGHT | 10 |
| WS2812B RGB LED | 8 |

Motor switching uses a **ULN2803A** Darlington transistor array — inputs driven directly from GPIO 10/11, outputs switching the motor supply. See the datasheet in `docs/`.

## Software

Built with **ESP-IDF 5.5.3** (CMake build system).

### Dependencies

| Library | Source |
|---------|--------|
| `espressif/led_strip ^3.0.0` | IDF Component Registry (managed) |
| `st7735_driver` | Local component (`components/`) |

### Project Structure

```
.
├── main/
│   ├── main.c          # Application entry point, all tasks and logic
│   ├── mpu6050.c/h     # I2C driver for MPU-6050
│   ├── hc_sr04.c/h     # GPIO driver for HC-SR04
│   └── web_page.h      # Embedded HTML dashboard (single-file)
├── components/
│   └── st7735_driver/  # SPI driver for ST7735 TFT
├── docs/               # Component datasheets
└── CMakeLists.txt
```

### RTOS Tasks

| Task | Priority | Purpose |
|------|----------|---------|
| `app_main` | 3 | Sensor loop, display update, sleep management |
| `dist_task` | 4 | HC-SR04 polling every 200 ms |
| `btn_poll_task` | 5 | Sleep button debounce polling every 20 ms |

## Build & Flash

```bash
# Set up ESP-IDF environment
. $IDF_PATH/export.sh

# Configure (optional)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

## Usage

1. Power on the car. The onboard LED turns **green** and the TFT shows `ESP32-C6 ACCEL`.
2. The MPU-6050 is calibrated automatically on startup — keep the car **stationary** for ~2 seconds.
3. Connect to the Wi-Fi network:
   - **SSID**: `ASE-remote-car`
   - **Password**: `supersafepassword`
4. Open a browser and navigate to **`http://192.168.4.1`**.
5. Use the arrow pad on the dashboard to drive the car.

### Sleep Behaviour

| Event | Result |
|-------|--------|
| No motor command for 60 s | Light sleep (LED: blue). Press button or wait 60 s more. |
| 60 s in light sleep OR button press | Deep sleep (LED: red). Press button to wake. |
| Button press during normal operation | Deep sleep immediately. |

## Wi-Fi Configuration

Credentials are defined as macros in `main/main.c`:

```c
#define WIFI_SSID    "ASE-remote-car"
#define WIFI_PASS    "supersafepassword"
```

The AP allows up to 4 simultaneous client connections on channel 1 with WPA2-PSK.

## HTTP API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serves the HTML dashboard |
| `/data` | GET | Returns JSON: `{"x", "y", "z", "dist", "speed"}` |
| `/motor?dir=<cmd>` | GET | Motor command: `fwd`, `left`, `right`, `stop` |

## License

Academic project — University of Aveiro, MECT programme, Advanced Embedded Systems (ASE).