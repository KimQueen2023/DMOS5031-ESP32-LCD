/**
 * @file    dmos5031.h
 * @brief   DMOS5031 / DMOS5030A "Dragonfly" ToF depth sensor — low-level driver.
 *
 * ## Sensor Overview
 * The DMOS5031 is a 100×100 pixel indirect-Time-of-Flight (iToF) sensor
 * that measures depth per pixel in millimetres.  Communication is via SPI
 * (mode 0, 32-bit word, big-endian byte order, 10 MHz typical).
 *
 * ## SPI Protocol (32-bit word, big-endian)
 * Every register access uses 32-bit words with a host→device byte-swap:
 *   - Little-endian host → swap each word → big-endian on the wire
 *   - Write: TX[WR_ID(0) | ADDR | VALUE | DUMMY]   (4 words)
 *   - Read:  TX[RD_ID(1) | ADDR | DUMMY | DUMMY | DUMMY] → value in RX[4]
 *
 * ## Initialisation Stages
 * ### Stage-A — Boot chain (dmos5031_run_stage_a)
 *   1. Software reset              (4 writes to reset registers)
 *   2. Chip-ID check               (expect 0x2000FC00 at 0x00000000)
 *   3. Boot sequence               (4 magic values to boot registers)
 *   4. ISP init (two-phase cmd)    (0x8080: resolution, FPS, binning, …)
 *   5. ISP start                   (0x8081: begin streaming)
 *
 * ### Stage-B — Frame capture setup (dmos5031_stage_b_init)
 *   - Allocates DMA RX buffers (single, ~40 KB)
 *   - Installs READY rising-edge GPIO ISR
 *
 * ### Stage-C — Continuous capture (dmos5031_start_capture_task)
 *   - FreeRTOS task pinned to CPU 1 at priority 5
 *   - Waits for READY rising edge (interrupt + poll fallback)
 *   - Burst-reads frame from address 0x2000E3AC
 *   - Decodes 20-byte header + depth payload (uint16_t, mm, row-major)
 *   - Calls application frame callback
 *
 * ## Frame Format
 *   - Header:       20 bytes (magic 0xCC 0xA0, size, temp, exposure, …)
 *   - Depth data:   width×height×2 bytes (uint16_t little-endian, mm)
 *   - Total:        header + depth = 20 + 20000 = 20020 bytes (100×100 mode)
 *
 * ## GPIO Configuration (hardcoded — no Kconfig dependency)
 *   | Signal  | GPIO | Notes                         |
 *   |---------|------|-------------------------------|
 *   | MOSI    | 11   |                               |
 *   | MISO    | 10   |                               |
 *   | SCLK    | 9    |                               |
 *   | CS      | 8    |                               |
 *   | READY   | 4    | Rising-edge (NOT GPIO7 — strapping!) |
 *   | RESET   | -1   | Software reset only (unused)  |
 *   | PWR_EN  | -1   | Always powered (unused)       |
 *
 * SPI bus: SPI2_HOST (independent from LCD's SPI3_HOST)
 *
 * @see dmos5031.c       Stage-A implementation
 * @see dmos5031_frame.c Stage-B/C frame capture + decode
 */

#ifndef DMOS5031_H
#define DMOS5031_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/*  Hardcoded Pin / Timing Defaults                                    */
/* ================================================================== */

#define DMOS5031_SPI_HOST              SPI2_HOST      /**< Independent from LCD's SPI3 */
#define DMOS5031_SPI_QUEUE_SIZE        4
#define DMOS5031_RESET_DELAY_MS        5
#define DMOS5031_POST_RESET_DELAY_MS   10
#define DMOS5031_IDLE_RETRY_MAX        400            /**< Max polls for idle state    */
#define DMOS5031_IDLE_RETRY_DELAY_MS   5
#define DMOS5031_CMD_SETTLE_DELAY_MS   10

/* GPIO assignment — all within the available pool IO7–IO11 */
#define DMOS5031_PIN_NUM_MISO          10
#define DMOS5031_PIN_NUM_MOSI          11
#define DMOS5031_PIN_NUM_SCLK          9
#define DMOS5031_PIN_NUM_CS            8
#define DMOS5031_PIN_NUM_READY         4              /**< Frame-ready, rising edge.
                                                           GPIO4 (NOT GPIO7 — GPIO7 is
                                                           MTDI strapping pin; HIGH at boot
                                                           forces download mode!) */
#define DMOS5031_PIN_NUM_RESET         (-1)           /**< Software reset only       */
#define DMOS5031_PIN_NUM_POWER_EN      (-1)           /**< Always powered            */

#define DMOS5031_SPI_CLOCK_HZ          10000000       /**< 10 MHz (safe for long wires) */
#define DMOS5031_FRAME_CAPTURE_TIMEOUT_MS  3000
#define DMOS5031_READY_HIGH_TIMEOUT_MS     2000

/* ================================================================== */
/*  SPI Protocol Constants                                             */
/* ================================================================== */

#define DMOS5031_DUMMY_WORD            0xFFFFFFFFUL   /**< Filler word in SPI transactions */
#define DMOS5031_SPI_WRITE_ID          0x00000000UL   /**< SPI command: write              */
#define DMOS5031_SPI_READ_ID           0x00000001UL   /**< SPI command: read               */

/* ================================================================== */
/*  Register Addresses                                                 */
/* ================================================================== */

#define DMOS5031_CHIP_ID_REG           0x00000000UL   /**< RO: chip identifier             */
#define DMOS5031_CHIP_ID_EXPECT         0x2000FC00UL   /**< Expected chip ID value          */
#define DMOS5031_IDLE_REG              0x40003004UL   /**< RO: idle status                 */
#define DMOS5031_IDLE_EXPECT            0x12341234UL   /**< Idle register expected value    */
#define DMOS5031_STATUS_REG            0x40003000UL   /**< RO: command completion status   */
#define DMOS5031_DATA_REG              0x40007000UL   /**< RW: ISP command payload         */

/* Boot / reset sequence registers */
#define DMOS5031_BOOT_REG0             0x4000E000UL
#define DMOS5031_BOOT_REG1             0x4000E004UL
#define DMOS5031_BOOT_REG2             0x4000E008UL
#define DMOS5031_BOOT_REG3             0x4000E010UL
#define DMOS5031_RESET_REG0            0x4000E010UL
#define DMOS5031_RESET_REG1            0x4000E00CUL
#define DMOS5031_RESET_REG2            0x4000E00CUL
#define DMOS5031_RESET_REG3            0x4000E00CUL

/* Boot / reset magic values */
#define DMOS5031_BOOT_VAL0             0x80080082UL
#define DMOS5031_BOOT_VAL1             0x01FFFF00UL
#define DMOS5031_BOOT_VAL2             0x0000000FUL
#define DMOS5031_BOOT_VAL3             0xFFFFFFFFUL
#define DMOS5031_RESET_VAL0            0xFFFFFFFEUL
#define DMOS5031_RESET_VAL1            0x00000000UL
#define DMOS5031_RESET_VAL2            0x00000001UL
#define DMOS5031_RESET_VAL3            0x00000000UL

/* ISP commands */
#define DMOS5031_CMD_ISP_INIT_SET      0x8080U        /**< Two-phase ISP init command    */
#define DMOS5031_CMD_ISP_START_STOP    0x8081U        /**< Start / stop streaming       */

/* ================================================================== */
/*  ISP Parameter Enumerations                                         */
/* ================================================================== */

typedef enum {
    DMOS5031_OUTPUT_DEPTH_ONLY  = 0,    /**< Depth data only (no IR)     */
    DMOS5031_OUTPUT_DEPTH_IR    = 1,    /**< Depth + IR combined         */
} dmos5031_output_mode_t;

typedef enum {
    DMOS5031_OUT_IF_NONE        = 0,
    DMOS5031_OUT_IF_UART        = 1,
    DMOS5031_OUT_IF_SPI         = 2,    /**< SPI output (default)        */
    DMOS5031_OUT_IF_MIPI        = 4,
} dmos5031_output_if_t;

typedef enum {
    DMOS5031_BINNING_MODE0      = 0,    /**< 100×100 full resolution     */
    DMOS5031_BINNING_MODE1      = 1,    /**< 2×2 binning → 50×50         */
    DMOS5031_BINNING_MODE2      = 2,    /**< 4×4 binning → 25×25         */
} dmos5031_binning_mode_t;

typedef enum {
    DMOS5031_UART_4800          = 0,
    DMOS5031_UART_9600          = 1,
    DMOS5031_UART_57600         = 2,
    DMOS5031_UART_115200        = 3,
    DMOS5031_UART_230400        = 4,
    DMOS5031_UART_460800        = 5,
    DMOS5031_UART_921600        = 6,
} dmos5031_uart_bps_t;

#define DMOS5031_DEFAULT_FPS           10U
#define DMOS5031_DEFAULT_EXPOSURE_US   20000UL

/* ================================================================== */
/*  Frame Capture Constants (Stage B/C)                                */
/* ================================================================== */

#define DMOS5031_DATA_BASE_ADDRESS     0x20000000UL   /**< Sensor memory base            */
#define DMOS5031_DATA_OFFSET_ADDRESS   0x0000E3ACUL   /**< Frame data offset             */
#define DMOS5031_FRAME_HEADER_SIZE     20U             /**< Bytes before depth payload    */
#define DMOS5031_FRAME_INFO_SIZE       16U             /**< Header info field size        */
#define DMOS5031_FRAME_MAX_BYTES       40000U          /**< Worst-case payload buffer     */
#define DMOS5031_BURST_CHUNK_BYTES     4080U           /**< Max bytes per SPI burst read  */
#define DMOS5031_SPI_IO_MAX_WORDS      (((DMOS5031_BURST_CHUNK_BYTES + 3U) / 4U) + 4U)
#define DMOS5031_FRAME_HEAD1           0xCC            /**< Frame magic byte 1            */
#define DMOS5031_FRAME_HEAD2           0xA0            /**< Frame magic byte 2            */
#define DMOS5031_RX_BUF_COUNT          1               /**< Single RX buffer (saves 40 KB)*/
#define DMOS5031_CAPTURE_TASK_STACK    8192            /**< FreeRTOS stack for dmos_cap   */

/* ================================================================== */
/*  Initialisation Stage Tracker                                       */
/* ================================================================== */

typedef enum {
    DMOS5031_STAGE_NONE = 0,
    DMOS5031_STAGE_GPIO_INIT,
    DMOS5031_STAGE_SPI_INIT,
    DMOS5031_STAGE_SOFT_RESET,
    DMOS5031_STAGE_CHIP_ID_CHECK,
    DMOS5031_STAGE_IDLE_CHECK,
    DMOS5031_STAGE_BOOT_SEQUENCE,
    DMOS5031_STAGE_ISP_INIT,
    DMOS5031_STAGE_ISP_START,
    DMOS5031_STAGE_FRAME_CAPTURE,
    DMOS5031_STAGE_FRAME_DECODE,
} dmos5031_stage_t;

/* ================================================================== */
/*  Frame Structures                                                   */
/* ================================================================== */

/** @brief 20-byte frame header received from the sensor */
typedef struct __attribute__((packed)) {
    uint8_t head1;                  /**< 0xCC                     */
    uint8_t head2;                  /**< 0xA0                     */
    uint8_t data_len0_7;            /**< Total data length LSB    */
    uint8_t data_len8_15;           /**< Total data length MSB    */
    uint8_t command;                /**< Usually 0xFF = frame     */
    uint8_t output_mode;            /**< 0=depth only, 1=depth+IR */
    uint8_t temp_ext_raw;           /**< External temp (raw - 43 = °C) */
    uint8_t temp_int_raw;           /**< Internal temp (raw - 43 = °C) */
    uint8_t exposure_time0_7;       /**< Exposure [7:0]           */
    uint8_t exposure_time8_11;      /**< Exposure [11:8]          */
    uint8_t exposure_time12_19;     /**< Exposure [19:12]         */
    uint8_t exposure_time20_23;     /**< Exposure [23:20]         */
    uint8_t error_code;             /**< 0 = no error             */
    uint8_t reserved1;
    uint8_t frame_height;           /**< Pixels (after binning)   */
    uint8_t frame_width;            /**< Pixels (after binning)   */
    uint16_t frame_id;              /**< 12-bit rolling counter   */
    uint8_t isp_version;            /**< ISP firmware version     */
    uint8_t reserved3;
} dmos5031_frame_head_t;

/** @brief Decoded frame with parsed metadata */
typedef struct {
    uint8_t width;                  /**< Sensor columns            */
    uint8_t height;                 /**< Sensor rows               */
    uint8_t output_mode;            /**< Depth-only or depth+IR    */
    uint16_t frame_id;              /**< Rolling frame counter     */
    uint8_t error_code;             /**< Sensor error flags        */
    int8_t temp_int;                /**< Internal temperature  °C  */
    int8_t temp_ext;                /**< External temperature  °C  */
    uint32_t exposure_time;         /**< Integration time (µs)     */
    uint8_t isp_version;            /**< ISP firmware version      */
    uint16_t data_length;           /**< Total payload bytes       */
    uint16_t depth_data_size;       /**< Depth-only segment size   */
    uint16_t center_depth;          /**< Centre pixel depth (mm)   */
    bool ready;                     /**< Frame successfully decoded*/
} dmos5031_frame_t;

/** @brief ISP initialisation parameters (sent via cmd 0x8080) */
typedef struct __attribute__((packed)) {
    uint8_t fps;                    /**< Target frame rate (1–20)  */
    uint8_t out_if;                 /**< Output interface (SPI=2)  */
    uint8_t out_mode;               /**< Depth-only or depth+IR    */
    uint8_t roi_ul_x;               /**< ROI upper-left X (0–99)   */
    uint8_t roi_ul_y;               /**< ROI upper-left Y (0–99)   */
    uint8_t roi_br_x;               /**< ROI bottom-right X (0–99) */
    uint8_t roi_br_y;               /**< ROI bottom-right Y (0–99) */
    uint8_t binning_mode;           /**< 0=100×100, 1=50×50, 2=25×25 */
    uint8_t uart_bps;               /**< UART baud-rate index      */
    uint8_t reserved[2];
    uint8_t ap_confirm;             /**< 0=phase1, 1=phase2        */
} dmos5031_isp_init_set_t;

/* ================================================================== */
/*  Callback & Device Handle                                           */
/* ================================================================== */

/** @brief Frame-ready callback type.
 *  Called from the capture task (CPU 1, priority 5) when a new depth
 *  frame has been decoded.  Keep the callback short — heavy processing
 *  (LCD rendering) should be deferred to the LVGL timer (CPU 0). */
typedef struct dmos5031_dev dmos5031_dev_t;
typedef void (*dmos5031_frame_cb_t)(dmos5031_dev_t *dev, void *user_ctx);

/** @brief DMOS5031 device handle — all driver state in one struct */
typedef struct dmos5031_dev {
    /* SPI / pin configuration */
    spi_device_handle_t spi;
    spi_host_device_t   spi_host;
    int pin_miso, pin_mosi, pin_sclk, pin_cs;
    int pin_ready, pin_reset, pin_power_en;
    int spi_clock_hz;
    bool spi_bus_inited, spi_dev_inited;

    /* Stage-A results */
    uint32_t chip_id;
    uint32_t idle_reg_value;
    uint32_t status_reg_value;
    dmos5031_stage_t last_stage;
    esp_err_t last_error;
    dmos5031_isp_init_set_t isp_init_param;

    /* Stage-B/C runtime (allocated in dmos5031_stage_b_init) */
    dmos5031_frame_head_t frame_head;
    dmos5031_frame_t      frame;
    uint8_t  *rx_buffer[1];            /**< Single DMA RX buffer (~40 KB)  */
    uint8_t   rx_buf_count;
    uint8_t   rx_buf_index;
    uint16_t *depth_buffer;            /**< Decoded depth data (uint16_t mm) */
    uint32_t *spi_io_tx;               /**< Scratch TX buffer for burst reads */
    uint32_t *spi_io_rx;               /**< Scratch RX buffer for burst reads */
    size_t    rx_buffer_size;
    bool      stage_b_inited;

    /* Capture task (CPU 1, priority 5) */
    TaskHandle_t capture_task;
    volatile bool capture_task_run;

    /* Application callback (called when new frame is ready) */
    dmos5031_frame_cb_t frame_callback;
    void *frame_callback_ctx;
} dmos5031_dev_t;

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

/** @brief Initialise GPIO + SPI bus for the DMOS5031.
 *  @param dev  Zeroed device handle (caller-owned, static lifetime). */
esp_err_t dmos5031_init(dmos5031_dev_t *dev);

/** @brief Tear down SPI bus and release resources. */
esp_err_t dmos5031_deinit(dmos5031_dev_t *dev);

/** @brief Run Stage-A boot chain (soft reset → chip ID → boot → ISP init → ISP start).
 *  Must be called after dmos5031_init().  VCSEL should start blinking on success. */
esp_err_t dmos5031_run_stage_a(dmos5031_dev_t *dev);

/** @brief Stage-B: allocate frame buffers, install READY ISR.  Call after Stage-A. */
esp_err_t dmos5031_stage_b_init(dmos5031_dev_t *dev);

/** @brief Stage-B tear-down: stop capture task, free buffers. */
esp_err_t dmos5031_stage_b_deinit(dmos5031_dev_t *dev);

/** @brief Blocking single-frame capture (used for one-shot reads). */
esp_err_t dmos5031_capture_frame(dmos5031_dev_t *dev, uint32_t timeout_ms);

/** @brief Return pointer to the most recent decoded frame (or NULL). */
const dmos5031_frame_t *dmos5031_get_frame(const dmos5031_dev_t *dev);

/** @brief Print frame summary to serial (depth samples, temperature, …). */
void dmos5031_log_frame_summary(const dmos5031_dev_t *dev);

/** @brief Start continuous capture FreeRTOS task on CPU 1, priority 5. */
esp_err_t dmos5031_start_capture_task(dmos5031_dev_t *dev);

/** @brief Stop the capture task and clean up. */
esp_err_t dmos5031_stop_capture_task(dmos5031_dev_t *dev);

/** @brief Return human-readable name for an init stage. */
const char *dmos5031_stage_to_string(dmos5031_stage_t stage);

/** @brief Send an ISP command with payload to DATA_REG.  Low-level primitive. */
esp_err_t dmos5031_send_cmd(dmos5031_dev_t *dev, uint16_t cmd, const uint32_t *value, uint32_t words);

/** @brief Send a read command and read back the response from DATA_REG.
 *  @param cmd    ISP command code (e.g. 0x9003, 0x908A).
 *  @param out    [in/out] Payload for the command / receives response data.
 *  @param words  Number of uint32_t words in the response payload.
 *  @return ESP_OK on success. */
esp_err_t dmos5031_get_cmd_value(dmos5031_dev_t *dev, uint16_t cmd, uint32_t *out, uint32_t words);

#ifdef __cplusplus
}
#endif

#endif /* DMOS5031_H */
