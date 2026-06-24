# DMOS5031 ToF Depth Heatmap on ESP32-S3 + ST7789 LCD

Real-time 100×100 depth heatmap display using a DMOS5031 Time-of-Flight sensor
and a 172×320 ST7789 TFT LCD driven by an ESP32-S3 microcontroller.

![System](docs/block_diagram.png)

## Features

- **100×100 depth heatmap** at ~10 FPS with perceptually uniform colour mapping
- **Direct LCD rendering** via LVGL canvas + full-screen double-buffered SPI flush
- **Dual-core architecture**: CPU 0 dedicated to LVGL rendering, CPU 1 to sensor I/O
- **Diagnostic overlay**: centre depth (mm), sensor temperature, frame rate, draw timing
- **Minimal dependencies** — no WiFi, BLE, SD card, or IMU required

## Hardware

| Component | Part | Interface | Notes |
|-----------|------|-----------|-------|
| MCU | ESP32-S3 | — | Dual-core Xtensa LX7 @ 240 MHz, octal PSRAM |
| LCD | ST7789 172×320 TFT | SPI3 (12 MHz, mode 0) | 16-bit RGB565 colour |
| ToF Sensor | DMOS5031 / DMOS5030A | SPI2 (10 MHz, mode 0) | 100×100 pixels, 0–8 m range |
| Power | USB-C or LiPo | — | Battery voltage monitored via ADC |

## GPIO Wiring

### DMOS5031 ToF Sensor (SPI2)

| Sensor Pin | ESP32-S3 GPIO | Signal |
|------------|---------------|--------|
| MOSI | **IO11** | SPI data (host → sensor) |
| MISO | **IO10** | SPI data (sensor → host) |
| SCLK | **IO9** | SPI clock |
| CS | **IO8** | SPI chip select |
| DATA_READY | **IO4** | Frame-ready (rising edge) |
| RESET | *not connected* | Software reset used instead |
| VCC | 3.3V / 5V | Sensor power |
| GND | GND | Common ground |

### ST7789 LCD (SPI3) — pre-wired on the dev board

| LCD Pin | ESP32-S3 GPIO |
|---------|---------------|
| SCLK | 40 |
| MOSI | 45 |
| DC | 41 |
| CS | 42 |
| RST | 39 |
| BL | 46 (LEDC PWM) |

> **Important**: IO7–IO11 are the only free GPIOs on this board. Other pins
> (IO0–IO1, IO14–IO21, IO38–IO48) are occupied by the LCD, PSRAM, or other
> onboard peripherals.

## Architecture

### Task / Core Layout

```
CPU 0 (LVGL rendering, never preempted)    CPU 1 (sensor I/O + background)
─────────────────────────────────────      ───────────────────────────────
main loop (pri 1)                          dmos_cap (pri 5)
  ├─ lv_timer_handler() every 2 ms         ├─ wait READY rising edge
  ├─ LVGL timer @ 50 ms → heatmap update   ├─ SPI2 burst-read frame
  │   ├─ colour-map 10K depth values       ├─ decode header + depth payload
  │   ├─ write lv_canvas buffer            └─ frame callback → set flag
  │   └─ update text labels
  └─ LVGL flush → SPI3 LCD write           Driver task (pri 3)
                                               └─ battery ADC @ 10 Hz
```

### Rendering Pipeline

```
Sensor frame arrives                         ~100 ms (10 FPS)
    │
    ▼
Capture task sets g_new_frame = true         < 1 µs (volatile store)
    │
    ▼
LVGL 50 ms timer fires                       colour-map 10K depth values
    │                                         via 4001-entry LUT (~1 ms)
    ▼
Canvas buffer written + invalidated          lv_obj_invalidate()
    │
    ▼
lv_timer_handler() renders dirty area        ~2 partial flushes × 22 ms SPI
    │                                         = ~44 ms total SPI time
    ▼
LCD updated                                  100×100 heatmap visible
```

**Total latency**: sensor frame → LCD pixel < 100 ms (1 frame behind)

### Colour Map

Depth values (uint16_t, millimetres) are mapped to a 5-stop piecewise-linear gradient:

| Depth (mm) | Colour | Meaning |
|-----------|--------|---------|
| 0 | Black | No data / error |
| < 200 | Red | Very near |
| 200–500 | Orange → Yellow | Near |
| 500–1000 | Yellow → Green | Medium |
| 1000–2000 | Green → Cyan | Far |
| 2000–4000 | Cyan → Blue | Very far |
| > 4000 | Dark blue | Maximum range |

The LUT is precomputed once at startup (4001 entries × 2 bytes = 8 KB, stored in PSRAM).

## Build

### Prerequisites

- **ESP-IDF v5.4** (tested; v5.1+ should also work)
  - Install path: `D:/esp32/test_esp32_dmos5031/esp-idf-v5.4/esp-idf-v5.4`
  - Tools path: `D:/esp32/.espressif_v54/`
  - Set `IDF_TOOLS_PATH=D:/esp32/.espressif_v54` before building
- **Python 3.12** with ESP-IDF virtual environment

### Commands

```bash
# Set up ESP-IDF environment (Windows PowerShell)
. D:/esp32/test_esp32_dmos5031/esp-idf-v5.4/esp-idf-v5.4/export.ps1

# Navigate to project
cd D:/esp32/ESP32-S3-LCD-1.47B-Demo/ESP32-S3-LCD-1.47B-Test

# Configure target
idf.py set-target esp32s3

# Build
idf.py build

# Flash (replace COMx with your serial port)
idf.py -p COMx flash

# Monitor serial output
idf.py -p COMx monitor
```

### sdkconfig Key Settings

```
CONFIG_SPIRAM=y                  # Octal PSRAM enabled
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_LV_MEM_SIZE_KILOBYTES=96  # LVGL internal heap (96 KB)
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
CONFIG_LV_USE_PERF_MONITOR=y     # LVGL performance counters
```

## Project Structure

```
main/
├── main.c                     # Entry point: init sequence, main LVGL loop
├── CMakeLists.txt             # Component registration (minimal deps)
│
├── DMOS5031/                  # DMOS5031 ToF sensor driver
│   ├── dmos5031.h             #   Register map, structs, API declarations
│   ├── dmos5031.c             #   Stage-A: GPIO/SPI init + boot chain
│   └── dmos5031_frame.c       #   Stage-B/C: READY ISR + frame capture + decode
│
├── LVGL_UI/
│   ├── LVGL_Heatmap.h         #   Heatmap display API
│   └── LVGL_Heatmap.c         #   Colour LUT, LVGL canvas, render timer
│
├── LVGL_Driver/
│   ├── LVGL_Driver.h          #   LVGL buffer configuration
│   └── LVGL_Driver.c          #   LVGL display driver, flush callback, tick
│
├── LCD_Driver/                #   ST7789 LCD driver (from original demo)
│   ├── ST7789.h / .c          #   SPI3 bus init, panel creation, backlight
│   └── Vernon_ST7789T/        #   Low-level ST7789T panel driver
│
├── BAT_Driver/                #   Battery voltage ADC (from original demo)
│   └── BAT_Driver.h / .c
│
└── Button_Driver/             #   Boot button (from original demo, not used)
    └── Button_Driver.h / .c

components/                    # ESP-IDF component manager dependencies
├── lvgl__lvgl/                #   LVGL 8.3 (Light and Versatile Graphics Library)
└── espressif__led_strip/      #   LED strip driver (not actively used)
```

## Known Issues & Troubleshooting

### Watchdog reset / LCD freezes at checkerboard
**Symptom**: `task_wdt: CPU 0: main` — LCD stuck at initial checkerboard pattern.
**Cause**: 110 KB full-screen LVGL draw buffer exceeds SPI DMA transfer limit.
**Fix**: Use half-screen buffer (55 KB).  Defined as `LVGL_BUF_SIZE` in `LVGL_Driver.h`.

### chip_id reads 0xFFFFFFFF
**Symptom**: `DMOS5031: chip_id=0xFFFFFFFF, expect=0x2000FC00`.
**Cause**: MISO pin floating (sensor not responding).
**Check**:
1. Sensor powered?  (measure VCC/GND)
2. SPI pins correct?  (MISO=IO10, MOSI=IO11, SCLK=IO9, CS=IO8)
3. VCSEL blinking?  (visible red flicker through phone camera)

### LCD refresh rate is low (3–4 seconds per frame)
**Symptom**: FPS shows 0–1, heatmap updates very slowly.
**Cause 1**: Sensor delivering <1 FPS (check VCSEL blink rate).
**Cause 2**: LVGL partial-render buffer too small (was 5.5 KB → fixed to 55 KB).

### Serial logs flooded with frame data
Edit `dmos5031_frame.c` and comment out `dmos5031_log_frame_summary(dev)`.
Use VCSEL blinking to judge sensor health instead.

## Performance

| Metric | Value |
|--------|-------|
| Sensor frame rate | ~10 FPS (configurable 1–20) |
| LCD refresh rate | ~10 FPS (limited by sensor) |
| SPI2 clock (sensor) | 10 MHz |
| SPI3 clock (LCD) | 12 MHz |
| Render time per frame | ~1 ms colour-mapping + ~44 ms SPI flush |
| CPU usage | < 5% (mostly idle during SPI DMA) |
| Free heap | ~200 KB (internal SRAM) |
| Binary size | ~500 KB (84% app partition free) |

## License

Based on the ESP32-S3-LCD-1.47B demo project and the DM5031 ESP32 SDK.
SPDX-License-Identifier: CC0-1.0
