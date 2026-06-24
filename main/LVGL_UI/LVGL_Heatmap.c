/**
 * @file LVGL_Heatmap.c
 * @brief DMOS5031 100×100 depth heatmap using LVGL canvas (DMA SRAM buffer).
 *
 * All LCD access happens in LVGL task context (lv_timer) to avoid
 * corrupting LVGL's async flush state machine.
 */
#include "LVGL_Heatmap.h"
#include "LVGL_Driver.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "HEATMAP";

/* ================================================================== */
/*  Colour LUT                                                        */
/* ================================================================== */
#define DEPTH_LUT_SIZE      4001
#define DEPTH_CLAMP_MAX     4000
static uint16_t *s_depth_lut;

typedef struct { uint16_t depth_mm; uint8_t r, g, b; } color_stop_t;

static const color_stop_t s_stops[] = {
    {    0,   0,   0,   0 }, {  200, 255,   0,   0 },
    {  500, 255, 165,   0 }, { 1000, 255, 255,   0 },
    { 2000,   0, 255,   0 }, { 4000,   0,   0, 255 },
};
#define NUM_STOPS  (sizeof(s_stops) / sizeof(s_stops[0]))

static inline uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{ return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)); }

static void lerp_color(const color_stop_t *a, const color_stop_t *b,
                       uint16_t d, uint8_t *r, uint8_t *g, uint8_t *bo)
{
    float t = (float)(d - a->depth_mm) / (float)(b->depth_mm - a->depth_mm);
    *r  = (uint8_t)(a->r + t * (b->r - a->r));
    *g  = (uint8_t)(a->g + t * (b->g - a->g));
    *bo = (uint8_t)(a->b + t * (b->b - a->b));
}

static void depth_lut_init(void)
{
    if (s_depth_lut) return;
    s_depth_lut = heap_caps_malloc(DEPTH_LUT_SIZE * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_depth_lut) { ESP_LOGE(TAG, "LUT alloc failed"); return; }
    size_t seg = 0;
    for (int d = 0; d < DEPTH_LUT_SIZE; d++) {
        while (seg < NUM_STOPS - 1 && d > s_stops[seg + 1].depth_mm) seg++;
        uint16_t c;
        if (d == 0) c = 0x0000;
        else if (d >= s_stops[NUM_STOPS - 1].depth_mm)
            c = rgb888_to_rgb565(s_stops[NUM_STOPS - 1].r, s_stops[NUM_STOPS - 1].g, s_stops[NUM_STOPS - 1].b);
        else { uint8_t r, g, b; lerp_color(&s_stops[seg], &s_stops[seg + 1], (uint16_t)d, &r, &g, &b); c = rgb888_to_rgb565(r, g, b); }
        s_depth_lut[d] = c;
    }
}

static inline uint16_t depth_to_rgb565(uint16_t mm) {
    if (mm >= DEPTH_LUT_SIZE) mm = DEPTH_CLAMP_MAX;
    return s_depth_lut ? s_depth_lut[mm] : 0;
}

/* ================================================================== */
/*  Module state                                                      */
/* ================================================================== */
#define SENSOR_W     100            /**< Sensor native resolution         */
#define SENSOR_H     100
#define DISPLAY_SZ   160            /**< Display size via LVGL zoom        */
#define ZOOM_VAL     410            /**< LVGL zoom: 256=1×, 410≈1.6×       */

static dmos5031_dev_t  *s_depth_dev;
static volatile bool    g_new_frame;

static lv_obj_t   *heatmap_canvas;
static lv_obj_t   *label_center;
static lv_obj_t   *label_temp;
static lv_obj_t   *label_fps;
static lv_obj_t   *label_c5x5;       /**< Center 5×5 average depth (mm)   */
static lv_timer_t *heatmap_timer;

static lv_color_t *canvas_buf;       /* DISPLAY_SZ² RGB565 (DMA SRAM)     */
static int         frame_count;
static uint32_t    last_sec_ts;
static int         last_fps;

/* ================================================================== */
/*  LVGL timer: update heatmap + labels (50 ms, LVGL task context)    */
/* ================================================================== */
static int cb_call_cnt = 0;
static int cb_skip_cnt = 0;

static void heatmap_update_cb(lv_timer_t *t)
{
    (void)t;
    cb_call_cnt++;
    if (!g_new_frame) { cb_skip_cnt++; return; }
    if (!s_depth_dev || !s_depth_dev->depth_buffer || !canvas_buf) return;

    if (!lvgl_port_lock(10)) {
        ESP_LOGW(TAG, "lvgl_port_lock TIMEOUT");
        return;
    }

    const dmos5031_frame_t *f = dmos5031_get_frame(s_depth_dev);
    if (!f || !f->ready) { lvgl_port_unlock(); return; }

    /* Colour-map 100×100 depth → canvas buffer (no scaling — LVGL zoom handles it) */
    int total = SENSOR_W * SENSOR_H;
    for (int i = 0; i < total; i++) {
        canvas_buf[i].full = depth_to_rgb565(s_depth_dev->depth_buffer[i]);
    }
    lv_obj_invalidate(heatmap_canvas);   /* ask LVGL to flush this area */

    /* Labels */
    char str[64];
    snprintf(str, sizeof(str), "Center: %u mm", f->center_depth);
    lv_label_set_text(label_center, str);
    snprintf(str, sizeof(str), "Temp: %d C", f->temp_int);
    lv_label_set_text(label_temp, str);
    /* Compute average of center 5×5 pixels (rows 48-52, cols 48-52) */
    uint32_t c5_sum = 0;
    for (int r = 48; r <= 52; r++)
        for (int c = 48; c <= 52; c++)
            c5_sum += s_depth_dev->depth_buffer[r * SENSOR_W + c];
    uint16_t c5_avg = (uint16_t)(c5_sum / 25);
    snprintf(str, sizeof(str), "Center 5x5=%umm", c5_avg);
    lv_label_set_text(label_c5x5, str);

    /* FPS */
    frame_count++;
    uint32_t now = xTaskGetTickCount();
    uint32_t elapsed = now - last_sec_ts;
    if (elapsed >= pdMS_TO_TICKS(1000)) {
        last_fps = (int)((uint64_t)frame_count * 1000 / pdTICKS_TO_MS(elapsed));
        frame_count = 0;
        last_sec_ts = now;
    }
    if (frame_count == 0) {  /* first frame after FPS reset → once per second */
        ESP_LOGI(TAG, "render: fps=%d cb_calls=%d cb_skips=%d",
                 last_fps, cb_call_cnt, cb_skip_cnt);
    }
    snprintf(str, sizeof(str), "FPS:%d", last_fps);
    lv_label_set_text(label_fps, str);

    g_new_frame = false;
    lvgl_port_unlock();
}

/* ================================================================== */
/*  Frame callback (capture task — just sets flag)                    */
/* ================================================================== */
static int cb_frame_cnt = 0;
void on_dmos5031_frame_cb(dmos5031_dev_t *dev, void *user_ctx)
{
    (void)dev; (void)user_ctx;
    g_new_frame = true;
    if (++cb_frame_cnt % 100 == 1) {
        ESP_LOGI(TAG, "frame_cb: cnt=%d", cb_frame_cnt);
    }
}

/* ================================================================== */
/*  Public API                                                        */
/* ================================================================== */

void Depth_Sensor_Attach(dmos5031_dev_t *dev)
{
    s_depth_dev = dev;
    dev->frame_callback = on_dmos5031_frame_cb;
    dev->frame_callback_ctx = NULL;
    frame_count = 0;
    last_sec_ts = xTaskGetTickCount();
    last_fps = 0;

    if (heatmap_timer == NULL) {
        heatmap_timer = lv_timer_create(heatmap_update_cb, 50, NULL);
    }
    ESP_LOGI(TAG, "Sensor attached");
}

void Depth_Hub_Create(lv_obj_t *parent)
{
    depth_lut_init();

    /* Canvas buffer in DMA SRAM: 100×100 sensor-native (20 KB).
     * LVGL renders it at DISPLAY_SZ×DISPLAY_SZ via lv_img_set_zoom(). */
    canvas_buf = heap_caps_malloc(SENSOR_W * SENSOR_H * sizeof(lv_color_t),
                                   MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!canvas_buf) {
        canvas_buf = heap_caps_malloc(SENSOR_W * SENSOR_H * sizeof(lv_color_t),
                                       MALLOC_CAP_8BIT);
    }
    if (!canvas_buf) { ESP_LOGE(TAG, "FATAL: canvas buf alloc failed"); return; }

    /* Fill checkerboard placeholder */
    for (int y = 0; y < SENSOR_H; y++)
        for (int x = 0; x < SENSOR_W; x++)
            canvas_buf[y * SENSOR_W + x].full = ((x / 4 + y / 4) & 1) ? 0xFFFF : 0x0000;

    /* Parent layout */
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    lv_obj_set_style_pad_all(parent, 4, 0);
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* Title */
    lv_obj_t *t = lv_label_create(parent);
    lv_label_set_text(t, "DOMI ToF + esp32");
    lv_obj_set_style_text_color(t, lv_color_white(), 0);

    /* Center depth */
    label_center = lv_label_create(parent);
    lv_label_set_text(label_center, "--- mm");
    lv_obj_set_style_text_color(label_center, lv_color_white(), 0);

    /* Heatmap canvas: 100×100 buffer, zoomed to DISPLAY_SZ×DISPLAY_SZ by LVGL.
     * This keeps the source buffer small (20 KB) while rendering large on screen.
     * lv_img_set_zoom uses nearest-neighbour by default for speed. */
    heatmap_canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(heatmap_canvas, canvas_buf, SENSOR_W, SENSOR_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_size(heatmap_canvas, SENSOR_W, SENSOR_H);
    lv_img_set_zoom(heatmap_canvas, ZOOM_VAL);           /* 1.6× upscale          */
    lv_img_set_antialias(heatmap_canvas, false);          /* nearest-neighbour     */
    lv_obj_set_style_border_width(heatmap_canvas, 2, 0);
    lv_obj_set_style_border_color(heatmap_canvas, lv_color_hex(0x444444), 0);

    /* FPS */
    label_fps = lv_label_create(parent);
    lv_label_set_text(label_fps, "FPS: --");
    lv_obj_set_style_text_color(label_fps, lv_color_white(), 0);

    /* Temp */
    label_temp = lv_label_create(parent);
    lv_label_set_text(label_temp, "Temp: -- C");
    lv_obj_set_style_text_color(label_temp, lv_color_white(), 0);

    /* Center 5×5 average depth */
    label_c5x5 = lv_label_create(parent);
    lv_label_set_text(label_c5x5, "Center 5*5 = ---mm");
    lv_obj_set_style_text_color(label_c5x5, lv_color_hex(0x00FF00), 0);

    ESP_LOGI(TAG, "Depth Hub ready: %dx%d canvas (%.1fx scale), DMA SRAM", DISPLAY_SZ, DISPLAY_SZ, (float)DISPLAY_SZ/SENSOR_W);
}
