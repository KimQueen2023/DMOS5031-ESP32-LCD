/**
 * @file dmos5031_frame.c
 * @brief Stage B/C: READY-triggered frame capture and depth-only decode.
 *
 * Adapted from DM5031_esp32_SDK — dmsr_frame output replaced with
 * frame callback, Kconfig references replaced with hardcoded values.
 */
#include <inttypes.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "dmos5031.h"

static const char *TAG = "DMOS5031_B";

/* ------------------------------------------------------------------ */
/*  READY interrupt support                                           */
/* ------------------------------------------------------------------ */
static SemaphoreHandle_t s_ready_sem;
static volatile bool s_ready_edge_seen;

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */
static uint8_t *frame_active_rx(const dmos5031_dev_t *dev)
{
    if (dev->rx_buf_count == 0) {
        return NULL;
    }
    return dev->rx_buffer[dev->rx_buf_index];
}

static uint32_t frame_swap_u32(uint32_t value)
{
    return ((value << 24) & 0xFF000000UL) |
           ((value << 8)  & 0x00FF0000UL) |
           ((value >> 8)  & 0x0000FF00UL) |
           ((value >> 24) & 0x000000FFUL);
}

static esp_err_t frame_spi_txrx(dmos5031_dev_t *dev, const void *tx_buf, void *rx_buf, size_t len)
{
    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    return spi_device_transmit(dev->spi, &trans);
}

/* ------------------------------------------------------------------ */
/*  Multi-word register read (for frame burst)                        */
/* ------------------------------------------------------------------ */
static esp_err_t frame_reg_read_multi(dmos5031_dev_t *dev, uint32_t addr, uint32_t *data, uint32_t words)
{
    if (words == 0 || (words + 4) > DMOS5031_SPI_IO_MAX_WORDS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->spi_io_tx == NULL || dev->spi_io_rx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t *tx = dev->spi_io_tx;
    uint32_t *rx = dev->spi_io_rx;
    memset(tx, 0, (words + 4) * sizeof(uint32_t));
    tx[0] = frame_swap_u32(DMOS5031_SPI_READ_ID);
    tx[1] = frame_swap_u32(addr);
    tx[2] = frame_swap_u32(DMOS5031_DUMMY_WORD);
    tx[3] = frame_swap_u32(DMOS5031_DUMMY_WORD);
    for (uint32_t i = 0; i < words; i++) {
        tx[i + 4] = DMOS5031_DUMMY_WORD;
    }

    esp_err_t ret = frame_spi_txrx(dev, tx, rx, (words + 4) * sizeof(uint32_t));
    if (ret != ESP_OK) {
        return ret;
    }
    for (uint32_t i = 0; i < words; i++) {
        data[i] = frame_swap_u32(rx[i + 4]);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Burst read (chunked at DMOS5031_BURST_CHUNK_BYTES)                */
/* ------------------------------------------------------------------ */
static esp_err_t frame_burst_read(dmos5031_dev_t *dev, uint32_t src_addr, void *dst, uint32_t bytes)
{
    uint8_t *dst_u8 = (uint8_t *)dst;
    while (bytes > 0) {
        uint32_t chunk = (bytes > DMOS5031_BURST_CHUNK_BYTES) ? DMOS5031_BURST_CHUNK_BYTES : bytes;
        uint32_t words = (chunk + 3U) / 4U;
        ESP_RETURN_ON_ERROR(frame_reg_read_multi(dev, src_addr, (uint32_t *)dst_u8, words), TAG, "burst read failed");
        src_addr += chunk;
        dst_u8   += chunk;
        bytes    -= chunk;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  READY GPIO ISR                                                    */
/* ------------------------------------------------------------------ */
static void IRAM_ATTR ready_gpio_isr(void *arg)
{
    (void)arg;
    s_ready_edge_seen = true;
    if (s_ready_sem != NULL) {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s_ready_sem, &woken);
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Wait for READY rising edge (interrupt + poll fallback)            */
/* ------------------------------------------------------------------ */
static esp_err_t frame_wait_ready_high(dmos5031_dev_t *dev, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t limit = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < limit) {
        if (gpio_get_level(dev->pin_ready)) {
            return ESP_OK;
        }
        if (s_ready_sem != NULL && xSemaphoreTake(s_ready_sem, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (gpio_get_level(dev->pin_ready)) {
                return ESP_OK;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t frame_wait_ready_low(dmos5031_dev_t *dev, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while (gpio_get_level(dev->pin_ready)) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "READY stayed high too long");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Frame decode: header + depth payload                              */
/* ------------------------------------------------------------------ */
static esp_err_t frame_decode(dmos5031_dev_t *dev)
{
    dmos5031_frame_head_t *head  = &dev->frame_head;
    dmos5031_frame_t      *frame = &dev->frame;
    uint8_t *rx = frame_active_rx(dev);

    dev->last_stage = DMOS5031_STAGE_FRAME_DECODE;
    if (head->head1 != DMOS5031_FRAME_HEAD1 || head->head2 != DMOS5031_FRAME_HEAD2) {
        ESP_LOGE(TAG, "frame header error: 0x%02X 0x%02X", head->head1, head->head2);
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(frame, 0, sizeof(*frame));
    frame->width          = head->frame_width;
    frame->height         = head->frame_height;
    frame->output_mode    = head->output_mode;
    frame->frame_id       = head->frame_id;
    frame->temp_int       = (int8_t)head->temp_int_raw - 43;
    frame->temp_ext       = (int8_t)head->temp_ext_raw - 43;
    frame->error_code     = head->error_code;
    frame->exposure_time  = head->exposure_time0_7 +
                            ((uint32_t)head->exposure_time8_11  << 8) +
                            ((uint32_t)head->exposure_time12_19 << 12) +
                            ((uint32_t)head->exposure_time20_23 << 20);
    frame->isp_version    = head->isp_version;
    frame->data_length    = ((uint16_t)head->data_len8_15 << 8) | head->data_len0_7;

    if (frame->output_mode != DMOS5031_OUTPUT_DEPTH_ONLY) {
        ESP_LOGW(TAG, "output_mode=%u (expected depth_only=0), parsing as depth only", frame->output_mode);
    }

    if (frame->width == 0 || frame->height == 0) {
        ESP_LOGE(TAG, "invalid resolution: %ux%u", frame->width, frame->height);
        return ESP_ERR_INVALID_SIZE;
    }

    frame->depth_data_size = (uint16_t)(frame->width * frame->height * sizeof(uint16_t));
    size_t payload_len = frame->data_length - DMOS5031_FRAME_INFO_SIZE;
    if (payload_len < frame->depth_data_size || payload_len > dev->rx_buffer_size || rx == NULL) {
        ESP_LOGE(TAG, "payload length invalid: payload=%u depth=%u data_len=%u",
                 (unsigned)payload_len, frame->depth_data_size, frame->data_length);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(dev->depth_buffer, rx, frame->depth_data_size);

    uint8_t cx = frame->width / 2;
    uint8_t cy = frame->height / 2;
    frame->center_depth = dev->depth_buffer[frame->width * cy + cx];
    frame->ready = true;
    return ESP_OK;
}

/* ================================================================== */
/*  Public API                                                        */
/* ================================================================== */

esp_err_t dmos5031_stage_b_init(dmos5031_dev_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG, "dev is null");
    ESP_RETURN_ON_FALSE(dev->spi_dev_inited, ESP_ERR_INVALID_STATE, TAG, "call init and Stage-A first");

    if (dev->stage_b_inited) {
        return ESP_OK;
    }

    /* Single RX buffer (no ping-pong, saves 40 KB DMA RAM) */
    dev->rx_buffer_size = DMOS5031_FRAME_MAX_BYTES;
    dev->rx_buf_count   = 1;
    dev->rx_buf_index   = 0;
    dev->rx_buffer[0]   = heap_caps_malloc(dev->rx_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(dev->rx_buffer[0] != NULL, ESP_ERR_NO_MEM, TAG, "frame RX buffer alloc failed");

    dev->depth_buffer = heap_caps_malloc(dev->rx_buffer_size, MALLOC_CAP_8BIT);
    dev->spi_io_tx    = heap_caps_malloc(DMOS5031_SPI_IO_MAX_WORDS * sizeof(uint32_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    dev->spi_io_rx    = heap_caps_malloc(DMOS5031_SPI_IO_MAX_WORDS * sizeof(uint32_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(dev->depth_buffer && dev->spi_io_tx && dev->spi_io_rx,
                        ESP_ERR_NO_MEM, TAG, "frame buffer alloc failed");

    /* READY interrupt setup */
    if (s_ready_sem == NULL) {
        s_ready_sem = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_ready_sem != NULL, ESP_ERR_NO_MEM, TAG, "READY semaphore create failed");
    }
    xSemaphoreTake(s_ready_sem, 0);
    s_ready_edge_seen = false;

    gpio_set_intr_type((gpio_num_t)dev->pin_ready, GPIO_INTR_POSEDGE);
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(ret, TAG, "GPIO ISR service install failed");
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add((gpio_num_t)dev->pin_ready, ready_gpio_isr, NULL),
                        TAG, "READY ISR register failed");

    dev->capture_task      = NULL;
    dev->capture_task_run  = false;
    dev->stage_b_inited    = true;
    ESP_LOGI(TAG, "Stage-B init ok, depth_only, 1 RX buffer, READY rising on GPIO%d",
             dev->pin_ready);
    return ESP_OK;
}

esp_err_t dmos5031_stage_b_deinit(dmos5031_dev_t *dev)
{
    if (dev == NULL || !dev->stage_b_inited) {
        return ESP_OK;
    }

    dmos5031_stop_capture_task(dev);

    gpio_isr_handler_remove((gpio_num_t)dev->pin_ready);
    gpio_set_intr_type((gpio_num_t)dev->pin_ready, GPIO_INTR_DISABLE);

    for (uint8_t i = 0; i < DMOS5031_RX_BUF_COUNT; i++) {
        free(dev->rx_buffer[i]);
        dev->rx_buffer[i] = NULL;
    }
    free(dev->depth_buffer);
    free(dev->spi_io_tx);
    free(dev->spi_io_rx);
    dev->depth_buffer  = NULL;
    dev->spi_io_tx     = NULL;
    dev->spi_io_rx     = NULL;
    dev->rx_buf_count  = 0;
    dev->stage_b_inited = false;
    return ESP_OK;
}

esp_err_t dmos5031_capture_frame(dmos5031_dev_t *dev, uint32_t timeout_ms)
{
    uint8_t *rx_payload = NULL;

    ESP_RETURN_ON_FALSE(dev != NULL && dev->stage_b_inited, ESP_ERR_INVALID_STATE, TAG, "Stage-B not initialized");

    dev->last_stage  = DMOS5031_STAGE_FRAME_CAPTURE;
    dev->frame.ready = false;
    rx_payload = frame_active_rx(dev);
    ESP_RETURN_ON_FALSE(rx_payload != NULL, ESP_ERR_INVALID_STATE, TAG, "RX buffer not ready");

    ESP_LOGD(TAG, "Waiting for READY rising edge...");
    ESP_RETURN_ON_ERROR(frame_wait_ready_high(dev, timeout_ms), TAG, "READY wait timeout");

    const uint32_t frame_addr = DMOS5031_DATA_BASE_ADDRESS + DMOS5031_DATA_OFFSET_ADDRESS;
    ESP_RETURN_ON_ERROR(frame_burst_read(dev, frame_addr, &dev->frame_head, DMOS5031_FRAME_HEADER_SIZE),
                        TAG, "frame header read failed");

    if (dev->frame_head.head1 != DMOS5031_FRAME_HEAD1 || dev->frame_head.head2 != DMOS5031_FRAME_HEAD2) {
        ESP_LOGE(TAG, "frame magic error: 0x%02X 0x%02X", dev->frame_head.head1, dev->frame_head.head2);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t frame_len = (((uint16_t)dev->frame_head.data_len8_15 << 8) | dev->frame_head.data_len0_7) -
                         DMOS5031_FRAME_INFO_SIZE;
    if (frame_len == 0 || frame_len > dev->rx_buffer_size) {
        ESP_LOGE(TAG, "frame payload length invalid: %u", frame_len);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_RETURN_ON_ERROR(frame_burst_read(dev, frame_addr + DMOS5031_FRAME_HEADER_SIZE, rx_payload, frame_len),
                        TAG, "frame payload read failed");

    if (!gpio_get_level(dev->pin_ready)) {
        ESP_LOGE(TAG, "READY went low during read, frame discarded");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(frame_decode(dev), TAG, "frame decode failed");
    ESP_RETURN_ON_ERROR(frame_wait_ready_low(dev, DMOS5031_READY_HIGH_TIMEOUT_MS), TAG, "READY low wait failed");

    return ESP_OK;
}

const dmos5031_frame_t *dmos5031_get_frame(const dmos5031_dev_t *dev)
{
    if (dev == NULL || !dev->frame.ready) {
        return NULL;
    }
    return &dev->frame;
}

void dmos5031_log_frame_summary(const dmos5031_dev_t *dev)
{
    const dmos5031_frame_t *f = dmos5031_get_frame(dev);
    if (f == NULL) {
        ESP_LOGW(TAG, "no valid frame");
        return;
    }

    ESP_LOGI(TAG,
             "frame summary: id=%u size=%ux%u mode=%u(len=%u) err=0x%02X exp=%" PRIu32 " temp=%d/%d isp=0x%02X",
             f->frame_id, f->width, f->height, f->output_mode, f->data_length,
             f->error_code, f->exposure_time, f->temp_int, f->temp_ext, f->isp_version);

    uint16_t p0     = dev->depth_buffer[0];
    uint16_t pc     = f->center_depth;
    uint16_t p_last = dev->depth_buffer[f->width * f->height - 1];
    uint16_t pmid   = dev->depth_buffer[f->width * (f->height / 2)];
    ESP_LOGI(TAG, "depth samples: center=%u p0=%u pmid=%u plast=%u", pc, p0, pmid, p_last);
}

/* ------------------------------------------------------------------ */
/*  Capture task (runs continuously, calls frame callback)            */
/* ------------------------------------------------------------------ */
static void dmos5031_capture_task_fn(void *arg)
{
    dmos5031_dev_t *dev = (dmos5031_dev_t *)arg;
    static uint16_t last_frame_id = 0xFFFF;

    ESP_LOGI(TAG, "capture task started");

    while (dev->capture_task_run) {
        esp_err_t ret = dmos5031_capture_frame(dev, DMOS5031_FRAME_CAPTURE_TIMEOUT_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "capture failed at %s: %s",
                     dmos5031_stage_to_string(dev->last_stage), esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        const dmos5031_frame_t *frame = dmos5031_get_frame(dev);
        if (frame != NULL && frame->frame_id != last_frame_id) {
            // dmos5031_log_frame_summary(dev);  /* silenced: judge VCSEL blinking instead */
            last_frame_id = frame->frame_id;

            /* Notify application via callback (e.g. set heatmap update flag) */
            if (dev->frame_callback) {
                dev->frame_callback(dev, dev->frame_callback_ctx);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    dev->capture_task = NULL;
    ESP_LOGI(TAG, "capture task stopped");
    vTaskDelete(NULL);
}

esp_err_t dmos5031_start_capture_task(dmos5031_dev_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL && dev->stage_b_inited, ESP_ERR_INVALID_STATE, TAG, "Stage-B not initialized");
    ESP_RETURN_ON_FALSE(dev->capture_task == NULL, ESP_ERR_INVALID_STATE, TAG, "capture task already running");

    /* Pin to CPU 1 so CPU 0 is free for LVGL (lv_timer_handler) */
    dev->capture_task_run = true;
    BaseType_t ok = xTaskCreatePinnedToCore(dmos5031_capture_task_fn,
                                "dmos_cap",
                                DMOS5031_CAPTURE_TASK_STACK,
                                dev,
                                5,
                                &dev->capture_task,
                                1);   /* CPU 1 */
    if (ok != pdPASS) {
        dev->capture_task_run = false;
        dev->capture_task = NULL;
        ESP_LOGE(TAG, "capture task create failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t dmos5031_stop_capture_task(dmos5031_dev_t *dev)
{
    if (dev == NULL || dev->capture_task == NULL) {
        return ESP_OK;
    }

    dev->capture_task_run = false;
    for (int i = 0; i < 50 && dev->capture_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (dev->capture_task != NULL) {
        vTaskDelete(dev->capture_task);
        dev->capture_task = NULL;
    }
    return ESP_OK;
}
