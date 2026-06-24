/**
 * @file    dragonfly.h
 * @brief   DMOS5031 / DMOS5030A "Dragonfly" ToF sensor — high-level API.
 *
 * Ported from Dragonfly_Linux_SDK_v1.6.0.0/src/dragonfly/dragonfly.h.
 * All functions operate on a dmos5031_dev_t handle (already initialised).
 *
 * ## Convention
 *   - Return 0 on success, -1 on failure.
 *   - "DragSet*" → write command  (calls dmos5031_send_cmd)
 *   - "DragGet*" → read  command  (calls dmos5031_get_cmd_value)
 *
 * ## Fixed-point formats used by LensCoeff
 *   - fx, fy, u0, v0 : u14p18  (divide by 2^18)
 *   - k1..k5_p2      : s5p27   (signed, divide by 2^27)
 *   - skew           : s8p24   (signed, divide by 2^24)
 */

#ifndef DRAGONFLY_H
#define DRAGONFLY_H

#include <stdint.h>
#include "dmos5031.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/*  ISP Command Codes (SPI / I2C)                                      */
/* ================================================================== */

#define DRAG_CMD_ISP_INIT_SET           0x8080U  /**< ISP init (two-phase)        */
#define DRAG_CMD_ISP_START_STOP         0x8081U  /**< Start / stop streaming      */
#define DRAG_CMD_AE_SET                 0x8082U  /**< AE enable + exposure time   */
#define DRAG_CMD_ROI_BINNING_SET        0x8083U  /**< ROI range + binning mode    */
#define DRAG_CMD_ISP_INIT_SOLIDIFY      0x8084U  /**< Save config to flash        */
#define DRAG_CMD_FPS_SET                0x8085U  /**< Set frame rate              */
#define DRAG_CMD_IR_LIMIT_SET           0x8087U  /**< Set IR limit                */
#define DRAG_CMD_AP_SYNC_SWITCH         0x8088U  /**< AP sync output control      */
#define DRAG_CMD_SOLIDIFY_ERASURE       0x808DU  /**< Erase solidified config     */
#define DRAG_CMD_AE_WINDOW_SET          0x800FU  /**< Set AE metering window      */
#define DRAG_CMD_ANTIMMI_SET            0x8019U  /**< Set Anti-MMI config         */
#define DRAG_CMD_SENSOR_INFO_GET        0x9003U  /**< Get lens coefficients       */
#define DRAG_CMD_FPS_GET                0x900AU  /**< Get current FPS             */
#define DRAG_CMD_AE_WINDOW_GET          0x9012U  /**< Get AE metering window      */
#define DRAG_CMD_ANTIMMI_GET            0x9019U  /**< Get Anti-MMI config         */
#define DRAG_CMD_IR_LIMIT_GET           0x908AU  /**< Get IR limit                */
#define DRAG_CMD_SOLIDIFY_CONFIG_GET    0x908CU  /**< Get solidified config       */
#define DRAG_CMD_OBST_DETECT_TRIGGER    0x9090U  /**< Obstacle detect trigger     */
#define DRAG_CMD_OBST_REF_PLANE_GET     0x9091U  /**< Get ref-plane result        */

/* ================================================================== */
/*  Data Structures                                                    */
/* ================================================================== */

/** @brief Lens calibration coefficients (44 bytes raw from sensor).
 *  Fixed-point formats: u14p18 for fx/fy/u0/v0, s5p27 for k1-k5_p2, s8p24 for skew. */
typedef struct __attribute__((packed)) {
    uint8_t  cali_mode;     /**< 0 = Normal, 1 = Fisheye                         */
    uint32_t fx;            /**< Focal length X, u14p18                           */
    uint32_t fy;            /**< Focal length Y, u14p18                           */
    uint32_t u0;            /**< Principal point X, u14p18                        */
    uint32_t v0;            /**< Principal point Y, u14p18                        */
    uint32_t k1;            /**< Radial distortion coeff 1, s5p27                  */
    uint32_t k2;            /**< Radial distortion coeff 2, s5p27                  */
    uint32_t k3;            /**< Radial distortion coeff 3, s5p27                  */
    uint32_t k4_p1;         /**< k4 (normal) / p1 (fisheye), s5p27                */
    uint32_t k5_p2;         /**< k5 (normal) / p2 (fisheye), s5p27                */
    uint32_t skew;          /**< Skew coefficient, s8p24                           */
} dragonfly_lens_coeff_t;

/** @brief Anti Multi-Module Interference configuration. */
typedef struct __attribute__((packed)) {
    uint8_t  mode;          /**< 0=off, 1=auto, 2=manual                         */
    uint8_t  devID;         /**< Device ID for manual mode                        */
    uint16_t reserved;
} dragonfly_antimmi_t;

/** @brief Auto-Exposure metering window (pixel coordinates). */
typedef struct __attribute__((packed)) {
    uint16_t x0, y0;        /**< Upper-left corner                                */
    uint16_t x1, y1;        /**< Lower-right corner                               */
} dragonfly_ae_window_t;

/** @brief Obstacle detection configuration. */
typedef struct __attribute__((packed)) {
    uint8_t  mode;          /**< Detection mode (0-6)                             */
    uint8_t  manu_times;    /**< Manual trigger times                             */
    uint8_t  reserved[2];
} dragonfly_obst_detect_t;

/** @brief Reference-plane collection result. */
typedef struct __attribute__((packed)) {
    uint32_t status;        /**< Collection status                                */
    uint32_t valid_point;   /**< Number of valid points                           */
} dragonfly_plane_result_t;

/* ================================================================== */
/*  Public API — All functions return 0 on success, -1 on error.       */
/* ================================================================== */

/* ---- Streaming control ---- */
int DragSetISPStart(dmos5031_dev_t *dev);
int DragSetISPStop(dmos5031_dev_t *dev);

/* ---- Frame rate ---- */
int DragSetFPS(dmos5031_dev_t *dev, int fps);
int DragGetFPS(dmos5031_dev_t *dev, int *fps);

/* ---- Exposure ---- */
int DragSetAEEn(dmos5031_dev_t *dev, int enable);
int DragSetExposureTime(dmos5031_dev_t *dev, int exposure_us);

/* ---- ROI & Binning ---- */
int DragSetROIAndBinningOff(dmos5031_dev_t *dev);
int DragSetROIRange(dmos5031_dev_t *dev, int x1, int y1, int x2, int y2);
int DragSetBiningMode(dmos5031_dev_t *dev, int mode);

/* ---- IR Limit ---- */
int DragSetIRLimit(dmos5031_dev_t *dev, int limit);
int DragGetIRLimit(dmos5031_dev_t *dev, int *limit);

/* ---- AP Sync ---- */
int DragSetAPSync(dmos5031_dev_t *dev, int enable);

/* ---- Anti-MMI ---- */
int DragSetAntiMMI(dmos5031_dev_t *dev, const dragonfly_antimmi_t *cfg);
int DragGetAntiMMI(dmos5031_dev_t *dev, dragonfly_antimmi_t *cfg);

/* ---- Lens calibration ---- */
int DragGetLensCoeff(dmos5031_dev_t *dev, dragonfly_lens_coeff_t *cali);
int DragGetCalib(dmos5031_dev_t *dev, float *calib_buf);

/* ---- AE window ---- */
int DragSetAEWindow(dmos5031_dev_t *dev, const dragonfly_ae_window_t *ae_roi);
int DragGetAEWindow(dmos5031_dev_t *dev, dragonfly_ae_window_t *ae_roi);

/* ---- Flash / solidify ---- */
int DragSetISPSolidifiedFlash(dmos5031_dev_t *dev);
int DragErasureISPSolidifiedConfig(dmos5031_dev_t *dev);
int DragGetISPSolidifiedConfig(dmos5031_dev_t *dev, dmos5031_isp_init_set_t *cfg);

/* ---- Obstacle detection ---- */
int DragSetObstacleDetectTrigger(dmos5031_dev_t *dev, const dragonfly_obst_detect_t *cfg);
int DragGetObstacleDetectRefPlaneResult(dmos5031_dev_t *dev, dragonfly_plane_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* DRAGONFLY_H */
