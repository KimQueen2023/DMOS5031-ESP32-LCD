/**
 * @file    LVGL_Heatmap.h
 * @brief   DMOS5031 depth heatmap display — LVGL canvas + text labels.
 *
 * ## Rendering Pipeline
 *
 *   Sensor frame arrives (CPU 1, capture task)
 *     │  on_dmos5031_frame_cb() — sets g_new_frame = true
 *     ▼
 *   LVGL timer fires at 50 ms (CPU 0, main task)
 *     │  heatmap_update_cb()
 *     │    → colour-maps 10 000 depth values via LUT
 *     │    → writes RGB565 pixels directly to lv_canvas buffer
 *     │    → lv_obj_invalidate() marks canvas dirty
 *     │    → updates text labels (centre depth, temp, FPS)
 *     ▼
 *   lv_timer_handler() (CPU 0 main loop, every 2 ms)
 *     │  LVGL renders dirty area to draw buffer (half-screen, 55 KB)
 *     │  → flush_cb → esp_lcd_panel_draw_bitmap() → SPI3 async
 *     │  → on_color_trans_done → lv_disp_flush_ready()
 *     ▼
 *   LCD updated
 *
 * ## Colour Map
 *   Depth 0–4000 mm mapped to a perceptually linear 5-stop gradient:
 *   black → red → orange → yellow → green → cyan → blue
 *   Stored as a 4001-entry LUT (8 KB, PSRAM) for O(1) lookup.
 *
 * ## Thread Safety
 *   - Frame callback runs on CPU 1 (capture task) — only sets a volatile flag.
 *   - All LVGL API calls happen on CPU 0 (main task) inside lvgl_port_lock.
 *   - No direct esp_lcd_panel_draw_bitmap() from the capture task to avoid
 *     corrupting LVGL's asynchronous flush state machine.
 */

#pragma once

#include "lvgl.h"
#include "dmos5031.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the depth heatmap UI on the given parent (usually lv_scr_act).
 *
 * Creates:
 *  - Title label ("DMOS5031 Depth")
 *  - Centre-depth label
 *  - 100×100 lv_canvas (checkerboard placeholder until first frame)
 *  - FPS label
 *  - Temperature label
 *  - Frame-ID label
 *
 * Call once at startup, before DMOS5031 capture begins.
 */
void Depth_Hub_Create(lv_obj_t *parent);

/**
 * @brief Attach a DMOS5031 device to the heatmap module.
 *
 * Registers on_dmos5031_frame_cb() as the frame-ready callback and
 * starts the 50 ms LVGL timer that drives the heatmap update cycle.
 *
 * Must be called after dmos5031_stage_b_init() and before
 * dmos5031_start_capture_task().
 */
void Depth_Sensor_Attach(dmos5031_dev_t *dev);

#ifdef __cplusplus
}
#endif
