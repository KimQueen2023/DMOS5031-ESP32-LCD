/**
 * @file dmos5031.c
 * @brief Stage-A: GPIO + SPI init, soft reset, chip ID, boot sequence, ISP init/start.
 *
 * Adapted from DM5031_esp32_SDK — Kconfig removed, GPIOs hardcoded,
 * binning set to 100×100, log level reduced.
 */
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "dmos5031.h"

static const char *TAG = "DMOS5031";

typedef struct __attribute__((packed)) {
    uint32_t size;
    uint16_t cmd;
    uint8_t cam_id;
    uint8_t checksum;
} dmos5031_msg_head_t;

typedef struct __attribute__((packed)) {
    dmos5031_msg_head_t head;
    uint32_t buffer[64];
} dmos5031_msg_body_t;

/* ------------------------------------------------------------------ */
/*  Byte-swap: host (little-endian) → device (big-endian)             */
/* ------------------------------------------------------------------ */
static uint32_t dmos5031_swap_u32(uint32_t value)
{
    return ((value << 24) & 0xFF000000UL) |
           ((value << 8)  & 0x00FF0000UL) |
           ((value >> 8)  & 0x0000FF00UL) |
           ((value >> 24) & 0x000000FFUL);
}

static uint8_t dmos5031_checksum8(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/* ------------------------------------------------------------------ */
/*  Low-level SPI transaction                                         */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_spi_txrx(dmos5031_dev_t *dev, const void *tx_buf, void *rx_buf, size_t len)
{
    spi_transaction_t trans = {
        .flags = 0,
        .length = len * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    return spi_device_transmit(dev->spi, &trans);
}

/* ------------------------------------------------------------------ */
/*  Single-register write / read                                      */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_reg_write(dmos5031_dev_t *dev, uint32_t addr, uint32_t value)
{
    uint32_t tx[4] = {
        dmos5031_swap_u32(DMOS5031_SPI_WRITE_ID),
        dmos5031_swap_u32(addr),
        dmos5031_swap_u32(value),
        dmos5031_swap_u32(DMOS5031_DUMMY_WORD),
    };

    ESP_LOGV(TAG, "write addr=0x%08" PRIX32 " value=0x%08" PRIX32, addr, value);
    return dmos5031_spi_txrx(dev, tx, NULL, sizeof(tx));
}

static esp_err_t dmos5031_reg_read(dmos5031_dev_t *dev, uint32_t addr, uint32_t *value)
{
    uint32_t tx[5] = {
        dmos5031_swap_u32(DMOS5031_SPI_READ_ID),
        dmos5031_swap_u32(addr),
        dmos5031_swap_u32(DMOS5031_DUMMY_WORD),
        dmos5031_swap_u32(DMOS5031_DUMMY_WORD),
        dmos5031_swap_u32(DMOS5031_DUMMY_WORD),
    };
    uint32_t rx[5] = {0};

    /* DEBUG: log raw TX */
    ESP_LOGI(TAG, "SPI_RD: tx=[%08lX %08lX %08lX %08lX %08lX]",
             tx[0], tx[1], tx[2], tx[3], tx[4]);

    esp_err_t ret = dmos5031_spi_txrx(dev, tx, rx, sizeof(tx));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI_RD: spi_txrx FAILED: %s", esp_err_to_name(ret));
        return ret;
    }

    /* DEBUG: log raw RX before byte-swap */
    ESP_LOGI(TAG, "SPI_RD: rx=[%08lX %08lX %08lX %08lX %08lX]",
             rx[0], rx[1], rx[2], rx[3], rx[4]);

    *value = dmos5031_swap_u32(rx[4]);
    ESP_LOGI(TAG, "read addr=0x%08" PRIX32 " value=0x%08" PRIX32, addr, *value);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Multi-register write (used by command send)                       */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_reg_write_multi(dmos5031_dev_t *dev, uint32_t addr, const uint32_t *data, uint32_t words)
{
    uint32_t tx[67] = {0};
    tx[0] = dmos5031_swap_u32(DMOS5031_SPI_WRITE_ID);
    tx[1] = dmos5031_swap_u32(addr);
    for (uint32_t i = 0; i < words; i++) {
        tx[i + 2] = dmos5031_swap_u32(data[i]);
    }
    tx[words + 2] = dmos5031_swap_u32(DMOS5031_DUMMY_WORD);
    return dmos5031_spi_txrx(dev, tx, NULL, (words + 3) * sizeof(uint32_t));
}

/* ------------------------------------------------------------------ */
/*  Wait for device idle                                              */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_wait_idle(dmos5031_dev_t *dev)
{
    uint32_t value = 0;
    dev->last_stage = DMOS5031_STAGE_IDLE_CHECK;
    for (int i = 0; i < DMOS5031_IDLE_RETRY_MAX; i++) {
        ESP_RETURN_ON_ERROR(dmos5031_reg_read(dev, DMOS5031_IDLE_REG, &value), TAG, "idle register read failed");
        dev->idle_reg_value = value;
        if (value == DMOS5031_IDLE_EXPECT) {
            return ESP_OK;
        }
        if (i == 0 || i == DMOS5031_IDLE_RETRY_MAX - 1) {
            ESP_LOGW(TAG, "idle_reg=0x%08" PRIX32 ", expect=0x%08" PRIX32 ", retry=%d",
                     value, DMOS5031_IDLE_EXPECT, i + 1);
        }
        vTaskDelay(pdMS_TO_TICKS(DMOS5031_IDLE_RETRY_DELAY_MS));
    }
    ESP_LOGE(TAG, "idle check failed, last=0x%08" PRIX32 ", expect=0x%08" PRIX32,
             value, DMOS5031_IDLE_EXPECT);
    return ESP_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------ */
/*  Software reset sequence                                           */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_soft_reset(dmos5031_dev_t *dev)
{
    static const uint32_t addrs[] = {
        DMOS5031_RESET_REG0, DMOS5031_RESET_REG1, DMOS5031_RESET_REG2, DMOS5031_RESET_REG3,
    };
    static const uint32_t values[] = {
        DMOS5031_RESET_VAL0, DMOS5031_RESET_VAL1, DMOS5031_RESET_VAL2, DMOS5031_RESET_VAL3,
    };

    dev->last_stage = DMOS5031_STAGE_SOFT_RESET;
    ESP_LOGI(TAG, "software reset start");
    for (size_t i = 0; i < 4; i++) {
        ESP_RETURN_ON_ERROR(dmos5031_reg_write(dev, addrs[i], values[i]), TAG, "software reset write failed");
    }
    ESP_LOGI(TAG, "software reset done");
    vTaskDelay(pdMS_TO_TICKS(DMOS5031_POST_RESET_DELAY_MS));
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Read and verify chip ID                                           */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_read_chip_id(dmos5031_dev_t *dev)
{
    uint32_t chip_id = 0;
    dev->last_stage = DMOS5031_STAGE_CHIP_ID_CHECK;

    for (int attempt = 1; attempt <= 5; attempt++) {
        esp_err_t ret = dmos5031_reg_read(dev, DMOS5031_CHIP_ID_REG, &chip_id);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "chip_id attempt %d/5: SPI error %s", attempt, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        dev->chip_id = chip_id;
        ESP_LOGI(TAG, "chip_id attempt %d/5: 0x%08" PRIX32 " (expect 0x%08" PRIX32 ")",
                 attempt, chip_id, DMOS5031_CHIP_ID_EXPECT);

        if (chip_id == DMOS5031_CHIP_ID_EXPECT) {
            ESP_LOGI(TAG, "chip_id MATCH on attempt %d", attempt);
            return ESP_OK;
        }
        if (chip_id == 0xFFFFFFFF) {
            ESP_LOGW(TAG, "ALL ONES — check: sensor powered? MISO wired to IO%d? CS=IO%d?", dev->pin_miso, dev->pin_cs);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGE(TAG, "chip_id FAILED after 5 attempts, last=0x%08" PRIX32, chip_id);
    return ESP_ERR_INVALID_RESPONSE;
}

/* ------------------------------------------------------------------ */
/*  Boot sequence                                                     */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_boot_sequence(dmos5031_dev_t *dev)
{
    static const uint32_t addrs[] = {
        DMOS5031_BOOT_REG0, DMOS5031_BOOT_REG1, DMOS5031_BOOT_REG2, DMOS5031_BOOT_REG3,
    };
    static const uint32_t values[] = {
        DMOS5031_BOOT_VAL0, DMOS5031_BOOT_VAL1, DMOS5031_BOOT_VAL2, DMOS5031_BOOT_VAL3,
    };

    dev->last_stage = DMOS5031_STAGE_BOOT_SEQUENCE;
    for (size_t i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "boot_step=%u addr=0x%08" PRIX32 " value=0x%08" PRIX32,
                 (unsigned)(i + 1), addrs[i], values[i]);
        ESP_RETURN_ON_ERROR(dmos5031_reg_write(dev, addrs[i], values[i]), TAG, "boot write failed");
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Check command status after ISP command                            */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_check_cmd_status(dmos5031_dev_t *dev, uint16_t cmd)
{
    uint32_t status = 0;
    ESP_RETURN_ON_ERROR(dmos5031_wait_idle(dev), TAG, "idle check after command failed");
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(dmos5031_reg_read(dev, DMOS5031_STATUS_REG, &status), TAG, "read command status failed");
    dev->status_reg_value = status;

    uint16_t err_code = ((cmd & 0xFF00U) == 0x8000U) ? (uint16_t)status : (uint16_t)(status & 0x00FFU);
    ESP_LOGI(TAG, "cmd=0x%04X status=0x%08" PRIX32 " err=0x%04X", cmd, status, err_code);
    ESP_RETURN_ON_FALSE(((status >> 16) == cmd) && (err_code == 0), ESP_FAIL, TAG, "command status invalid");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Multi-word register read (same address, burst)                     */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_reg_read_multi(dmos5031_dev_t *dev, uint32_t addr,
                                          uint32_t *data, uint32_t words)
{
    if (words == 0 || words > 32) return ESP_ERR_INVALID_ARG;

    uint32_t tx[36] = {0};   /* max 4 hdr + 32 data words */
    uint32_t rx[36] = {0};
    tx[0] = dmos5031_swap_u32(DMOS5031_SPI_READ_ID);
    tx[1] = dmos5031_swap_u32(addr);
    tx[2] = dmos5031_swap_u32(DMOS5031_DUMMY_WORD);
    tx[3] = dmos5031_swap_u32(DMOS5031_DUMMY_WORD);
    for (uint32_t i = 0; i < words; i++) {
        tx[i + 4] = DMOS5031_DUMMY_WORD;
    }

    esp_err_t ret = dmos5031_spi_txrx(dev, tx, rx, (words + 4) * sizeof(uint32_t));
    if (ret != ESP_OK) return ret;

    for (uint32_t i = 0; i < words; i++) {
        data[i] = dmos5031_swap_u32(rx[i + 4]);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Get-cmd-value: send read command + read back response              */
/* ------------------------------------------------------------------ */
esp_err_t dmos5031_get_cmd_value(dmos5031_dev_t *dev, uint16_t cmd,
                                         uint32_t *out, uint32_t words)
{
    /* Send the command (payload = out buffer, which sensor may ignore for reads) */
    ESP_RETURN_ON_ERROR(dmos5031_send_cmd(dev, cmd, out, words),
                        TAG, "get cmd send failed");

    /* Read back (words + 2) uint32_t from DATA_REG; response payload at offset 2 */
    uint32_t rx[34];
    uint32_t n = words + 2;
    ESP_RETURN_ON_ERROR(dmos5031_reg_read_multi(dev, DMOS5031_DATA_REG, rx, n),
                        TAG, "get cmd read-back failed");

    for (uint32_t i = 0; i < words; i++) {
        out[i] = rx[i + 2];
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Send ISP command via DATA_REG                                     */
/* ------------------------------------------------------------------ */
esp_err_t dmos5031_send_cmd(dmos5031_dev_t *dev, uint16_t cmd, const uint32_t *value, uint32_t words)
{
    dmos5031_msg_body_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.head.size = sizeof(dmos5031_msg_head_t) + words * sizeof(uint32_t);
    msg.head.cmd = cmd;
    msg.head.cam_id = 1;
    for (uint32_t i = 0; i < words; i++) {
        msg.buffer[i] = value ? value[i] : 0;
    }
    msg.head.checksum = dmos5031_checksum8((const uint8_t *)&msg.head, msg.head.size);

    ESP_RETURN_ON_ERROR(dmos5031_wait_idle(dev), TAG, "send cmd before idle check failed");
    ESP_RETURN_ON_ERROR(dmos5031_reg_write_multi(dev, DMOS5031_DATA_REG, (const uint32_t *)&msg, msg.head.size / 4U), TAG, "send cmd write failed");
    vTaskDelay(pdMS_TO_TICKS(DMOS5031_CMD_SETTLE_DELAY_MS));
    return dmos5031_check_cmd_status(dev, cmd);
}

/* ------------------------------------------------------------------ */
/*  ISP Init (two-phase command 0x8080)                                */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_isp_init(dmos5031_dev_t *dev)
{
    dev->last_stage = DMOS5031_STAGE_ISP_INIT;
    dev->isp_init_param.fps = 10;                          /* 10 FPS */
    dev->isp_init_param.out_if = DMOS5031_OUT_IF_SPI;
    dev->isp_init_param.out_mode = DMOS5031_OUTPUT_DEPTH_ONLY;
    dev->isp_init_param.roi_ul_x = 0;
    dev->isp_init_param.roi_ul_y = 0;
    dev->isp_init_param.roi_br_x = 99;
    dev->isp_init_param.roi_br_y = 99;
    dev->isp_init_param.binning_mode = DMOS5031_BINNING_MODE0;  /* 100×100 full resolution */
    dev->isp_init_param.uart_bps = DMOS5031_UART_115200;
    dev->isp_init_param.ap_confirm = 0;

    ESP_LOGI(TAG, "send DragISPInit stage1, out_mode=depth_only, binning=100x100");
    ESP_RETURN_ON_ERROR(dmos5031_send_cmd(dev, DMOS5031_CMD_ISP_INIT_SET,
                                          (const uint32_t *)&dev->isp_init_param,
                                          sizeof(dev->isp_init_param) / 4U),
                        TAG, "DragISPInit stage1 failed");

    dev->isp_init_param.ap_confirm = 1;
    ESP_LOGI(TAG, "send DragISPInit stage2, ap_confirm=1");
    return dmos5031_send_cmd(dev, DMOS5031_CMD_ISP_INIT_SET,
                             (const uint32_t *)&dev->isp_init_param,
                             sizeof(dev->isp_init_param) / 4U);
}

/* ------------------------------------------------------------------ */
/*  ISP Start (command 0x8081)                                        */
/* ------------------------------------------------------------------ */
static esp_err_t dmos5031_isp_start(dmos5031_dev_t *dev)
{
    uint32_t cmdval = 1;
    dev->last_stage = DMOS5031_STAGE_ISP_START;
    ESP_LOGI(TAG, "send DragSetISPStart");
    return dmos5031_send_cmd(dev, DMOS5031_CMD_ISP_START_STOP, &cmdval, 1);
}

/* ================================================================== */
/*  Public API                                                        */
/* ================================================================== */

const char *dmos5031_stage_to_string(dmos5031_stage_t stage)
{
    switch (stage) {
        case DMOS5031_STAGE_GPIO_INIT:    return "gpio_init";
        case DMOS5031_STAGE_SPI_INIT:     return "spi_init";
        case DMOS5031_STAGE_SOFT_RESET:   return "soft_reset";
        case DMOS5031_STAGE_CHIP_ID_CHECK:return "chip_id_check";
        case DMOS5031_STAGE_IDLE_CHECK:   return "idle_check";
        case DMOS5031_STAGE_BOOT_SEQUENCE:return "boot_sequence";
        case DMOS5031_STAGE_ISP_INIT:     return "isp_init";
        case DMOS5031_STAGE_ISP_START:    return "isp_start";
        case DMOS5031_STAGE_FRAME_CAPTURE:return "frame_capture";
        case DMOS5031_STAGE_FRAME_DECODE: return "frame_decode";
        default: return "none";
    }
}

esp_err_t dmos5031_init(dmos5031_dev_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG, "device pointer is null");
    memset(dev, 0, sizeof(*dev));

    /* Load hardcoded pin/config from header defines */
    dev->spi_host     = DMOS5031_SPI_HOST;
    dev->pin_miso     = DMOS5031_PIN_NUM_MISO;
    dev->pin_mosi     = DMOS5031_PIN_NUM_MOSI;
    dev->pin_sclk     = DMOS5031_PIN_NUM_SCLK;
    dev->pin_cs       = DMOS5031_PIN_NUM_CS;
    dev->pin_ready    = DMOS5031_PIN_NUM_READY;
    dev->pin_reset    = DMOS5031_PIN_NUM_RESET;
    dev->pin_power_en = DMOS5031_PIN_NUM_POWER_EN;
    dev->spi_clock_hz = DMOS5031_SPI_CLOCK_HZ;

    /* --- GPIO init --- */
    uint64_t gpio_mask = (1ULL << dev->pin_ready);
    if (dev->pin_reset >= 0)    gpio_mask |= (1ULL << dev->pin_reset);
    if (dev->pin_power_en >= 0) gpio_mask |= (1ULL << dev->pin_power_en);

    gpio_config_t io_conf = {
        .pin_bit_mask = gpio_mask,
        .mode = GPIO_MODE_INPUT,             /* READY is input-only; RESET/PWR_EN unused */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    dev->last_stage = DMOS5031_STAGE_GPIO_INIT;
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_set_direction(dev->pin_ready, GPIO_MODE_INPUT), TAG, "ready gpio set failed");
    if (dev->pin_reset >= 0) {
        ESP_RETURN_ON_ERROR(gpio_set_direction(dev->pin_reset, GPIO_MODE_OUTPUT), TAG, "reset gpio set failed");
        gpio_set_level(dev->pin_reset, 1);
    }
    if (dev->pin_power_en >= 0) {
        ESP_RETURN_ON_ERROR(gpio_set_direction(dev->pin_power_en, GPIO_MODE_OUTPUT), TAG, "power gpio set failed");
        gpio_set_level(dev->pin_power_en, 1);
    }
    ESP_LOGI(TAG, "gpio init ok, ready=%d reset=%d power=%d ready_level=%d",
             dev->pin_ready, dev->pin_reset, dev->pin_power_en, gpio_get_level(dev->pin_ready));

    /* --- SPI init (SPI2_HOST, independent from LCD's SPI3) --- */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = dev->pin_mosi,
        .miso_io_num = dev->pin_miso,
        .sclk_io_num = dev->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DMOS5031_FRAME_MAX_BYTES + 64,
    };
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = dev->spi_clock_hz,
        .mode = 0,
        .spics_io_num = dev->pin_cs,
        .queue_size = DMOS5031_SPI_QUEUE_SIZE,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    dev->last_stage = DMOS5031_STAGE_SPI_INIT;
    ESP_RETURN_ON_ERROR(spi_bus_initialize(dev->spi_host, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "spi bus init failed");
    dev->spi_bus_inited = true;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(dev->spi_host, &dev_cfg, &dev->spi), TAG, "spi add device failed");
    dev->spi_dev_inited = true;
    ESP_LOGI(TAG, "spi init ok, host=SPI2 clk=%d cs=%d mosi=%d miso=%d sclk=%d",
             dev->spi_clock_hz, dev->pin_cs, dev->pin_mosi, dev->pin_miso, dev->pin_sclk);

    return ESP_OK;
}

esp_err_t dmos5031_deinit(dmos5031_dev_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->spi_dev_inited) {
        spi_bus_remove_device(dev->spi);
        dev->spi_dev_inited = false;
    }
    if (dev->spi_bus_inited) {
        spi_bus_free(dev->spi_host);
        dev->spi_bus_inited = false;
    }
    return ESP_OK;
}

esp_err_t dmos5031_run_stage_a(dmos5031_dev_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG, "device pointer is null");

    ESP_RETURN_ON_ERROR(dmos5031_soft_reset(dev),    TAG, "A-stage failed at soft_reset");
    ESP_RETURN_ON_ERROR(dmos5031_read_chip_id(dev),  TAG, "A-stage failed at chip_id_check");
    ESP_RETURN_ON_ERROR(dmos5031_boot_sequence(dev), TAG, "A-stage failed at boot_sequence");
    ESP_RETURN_ON_ERROR(dmos5031_isp_init(dev),      TAG, "A-stage failed at isp_init");
    ESP_RETURN_ON_ERROR(dmos5031_isp_start(dev),     TAG, "A-stage failed at isp_start");

    ESP_LOGI(TAG, "A-stage boot chain OK");
    ESP_LOGI(TAG, "please check VCSEL blinking");
    return ESP_OK;
}
