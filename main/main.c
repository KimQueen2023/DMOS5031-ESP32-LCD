/**
 * @file    main.c
 * @brief   DMOS5031 ToF Depth Heatmap — ESP32-S3 + ST7789 172×320 LCD
 *
 * This firmware reads 100×100 depth frames from a DMOS5031 Time-of-Flight
 * sensor over SPI2, colour-maps the depth values (mm) to an RGB565 heatmap,
 * and displays the result on a 172×320 ST7789 TFT LCD via SPI3 using LVGL.
 *
 * ## Hardware
 *   - MCU:       ESP32-S3 (dual-core Xtensa LX7 @ 240 MHz)
 *   - LCD:       ST7789 172×320 TFT, SPI3_HOST (12 MHz)
 *   - Sensor:    DMOS5031 ToF, SPI2_HOST (10 MHz, mode 0, 32-bit words)
 *   - Memory:    Octal PSRAM enabled for LVGL draw buffers
 *
 * ## GPIO assignments
 *   | Signal | GPIO | Bus     |
 *   |--------|------|---------|
 *   | MOSI   | 11   | SPI2    |
 *   | MISO   | 10   | SPI2    |
 *   | SCLK   | 9    | SPI2    |
 *   | CS     | 8    | SPI2    |
 *   | READY  | 4    | GPIO in |
 *
 * ## Task / core layout
 *   | Task        | CPU | Pri | Role                    |
 *   |-------------|-----|-----|-------------------------|
 *   | main        | 0   | 1   | lv_timer_handler() loop |
 *   | dmos_cap    | 1   | 5   | SPI2 frame capture      |
 *   | Driver task | 1   | 3   | Battery ADC             |
 *
 *   CPU 0 is reserved for LVGL rendering (never preempted by sensor I/O).
 *   CPU 1 handles all SPI2 + background sensor work.
 *
 * ## Build
 *   idf.py set-target esp32s3 && idf.py build
 *
 * ## Flash
 *   idf.py -p <PORT> flash
 *
 * @author  Kim (based on ESP32-S3-LCD-1.47B demo + DM5031 ESP32 SDK)
 * @date    2026-06-08
 */

#include "ST7789.h"
#include "BAT_Driver.h"
#include "dmos5031.h"
#include "dragonfly.h"
#include "LVGL_Heatmap.h"
#include "esp_log.h"

/* ------------------------------------------------------------------ */
/*  Module-level state                                                */
/* ------------------------------------------------------------------ */

static const char *TAG_APP = "APP";
static dmos5031_dev_t s_dev;        /**< Single DMOS5031 device handle */

/* ------------------------------------------------------------------ */
/*  Background driver task (CPU 1, priority 3)                        */
/* ------------------------------------------------------------------ */

void Driver_Loop(void *parameter)
{
    while (1) {
        BAT_Get_Volts();                            /* sample battery ADC */
        vTaskDelay(pdMS_TO_TICKS(100));             /* 10 Hz              */
    }
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Entry point                                                       */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* Let power rails stabilize before any init (fixes intermittent cold-boot
     * failures where PSRAM or LCD reset isn't ready yet). */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* ---- Hardware init (minimal: LCD + battery only) ---- */
    BAT_Init();
    LCD_Init();                 /* SPI3, ST7789 panel, backlight PWM */
    LVGL_Init();                /* LVGL 8.3, half-screen double-buffer in PSRAM */

    /* ---- Create depth heatmap UI (the ONLY screen content) ---- */
    Depth_Hub_Create(lv_scr_act());

    /* ---- DMOS5031 ToF sensor init chain ---- */
    if (dmos5031_init(&s_dev) == ESP_OK) {
        ESP_LOGI(TAG_APP, "DMOS5031 init OK, running Stage-A...");
        if (dmos5031_run_stage_a(&s_dev) == ESP_OK) {
            /* Stage-A: soft reset → chip ID → boot seq → ISP init → ISP start */
            ESP_LOGI(TAG_APP, "DMOS5031 Stage-A OK");

            /* ---- Query calibration & IR limit (diagnostic print) ---- */
            {
                float calib[10];
                if (DragGetCalib(&s_dev, calib) == 0) {
                    ESP_LOGI(TAG_APP, "Calib: fx=%.2f fy=%.2f u0=%.2f v0=%.2f",
                             calib[0], calib[1], calib[2], calib[3]);
                    ESP_LOGI(TAG_APP, "Calib: k1=%.6f k2=%.6f k3=%.6f k4=%.6f k5=%.6f skew=%.6f",
                             calib[4], calib[5], calib[6], calib[7], calib[8], calib[9]);
                } else {
                    ESP_LOGW(TAG_APP, "DragGetCalib failed");
                }

                int ir_limit = 0;
                if (DragGetIRLimit(&s_dev, &ir_limit) == 0) {
                    ESP_LOGI(TAG_APP, "IR limit: %d", ir_limit);
                }
            }

            ESP_LOGI(TAG_APP, "initializing Stage-B...");
            if (dmos5031_stage_b_init(&s_dev) == ESP_OK) {
                /* Stage-B: allocate RX buffers, install READY ISR */
                Depth_Sensor_Attach(&s_dev);         /* register frame callback */
                dmos5031_start_capture_task(&s_dev); /* start capture on CPU 1 */
                ESP_LOGI(TAG_APP, "DMOS5031 capture task started");
            } else {
                ESP_LOGW(TAG_APP, "DMOS5031 Stage-B init failed");
            }
        } else {
            ESP_LOGW(TAG_APP, "DMOS5031 Stage-A failed");
        }
    } else {
        ESP_LOGW(TAG_APP, "DMOS5031 GPIO/SPI init failed");
    }

    /* ---- Background driver task (CPU 1, priority 3) ---- */
    xTaskCreatePinnedToCore(Driver_Loop, "Driver task", 4096, NULL, 3, NULL, 1);

    /* ---- Main LVGL rendering loop (CPU 0, priority 1) ---- */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2));               /* ~500 Hz scheduling */
        lv_timer_handler();                          /* process LVGL timers & rendering */
    }
}
