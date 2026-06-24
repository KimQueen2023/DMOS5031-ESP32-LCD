/**
 * @file    dragonfly.c
 * @brief   DMOS5031 "Dragonfly" high-level API — thin wrappers around
 *          dmos5031_send_cmd() / dmos5031_get_cmd_value().
 *
 * Ported from Dragonfly_Linux_SDK_v1.6.0.0/src/dragonfly/dragonfly.cpp.
 * All functions use the existing SPI2 command infrastructure.
 */
#include "dragonfly.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "DRAGONFLY";

/* ================================================================== */
/*  Streaming control                                                  */
/* ================================================================== */

int DragSetISPStart(dmos5031_dev_t *dev)
{
    uint32_t val = 1;
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_ISP_START_STOP, &val, 1);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragSetISPStop(dmos5031_dev_t *dev)
{
    uint32_t val = 0;
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_ISP_START_STOP, &val, 1);
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  Frame rate                                                         */
/* ================================================================== */

int DragSetFPS(dmos5031_dev_t *dev, int fps)
{
    if (fps < 1 || fps > 20) {
        ESP_LOGE(TAG, "DragSetFPS: invalid fps %d (1-20)", fps);
        return -1;
    }
    uint32_t val = (uint32_t)fps;
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_FPS_SET, &val, 1);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragGetFPS(dmos5031_dev_t *dev, int *fps)
{
    uint32_t val = 0;
    esp_err_t ret = dmos5031_get_cmd_value(dev, DRAG_CMD_FPS_GET, &val, 1);
    if (ret == ESP_OK && fps) *fps = (int)val;
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  Exposure                                                           */
/* ================================================================== */

int DragSetAEEn(dmos5031_dev_t *dev, int enable)
{
    /* AE set uses a 2-word struct: {ae_en, exposure_time} */
    uint32_t ae_set[2];
    ae_set[0] = (uint32_t)(enable ? 1 : 0);
    ae_set[1] = 0;   /* exposure_time not changed when only toggling AE */
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_AE_SET, ae_set, 2);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragSetExposureTime(dmos5031_dev_t *dev, int exposure_us)
{
    if (exposure_us < 128 || exposure_us > 80000) {
        ESP_LOGE(TAG, "DragSetExposureTime: invalid %d us (128-80000)", exposure_us);
        return -1;
    }
    /* Keep AE disabled when setting manual exposure */
    uint32_t ae_set[2];
    ae_set[0] = 0;                       /* AE off for manual exposure  */
    ae_set[1] = (uint32_t)exposure_us;
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_AE_SET, ae_set, 2);
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  ROI & Binning                                                      */
/* ================================================================== */

int DragSetROIAndBinningOff(dmos5031_dev_t *dev)
{
    /* Payload: {roi_binning_en=0, roi_ul_x, roi_ul_y, roi_br_x, roi_br_y, binning_mode, reserved[3]}
     * = 8 bytes = 2 uint32_t words */
    uint32_t buf[2] = {0};
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_ROI_BINNING_SET, buf, 2);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragSetROIRange(dmos5031_dev_t *dev, int x1, int y1, int x2, int y2)
{
    uint8_t raw[8] = {0};
    raw[0] = 1;                    /* roi_binning_en = ROI set         */
    raw[1] = (uint8_t)x1;
    raw[2] = (uint8_t)y1;
    raw[3] = (uint8_t)x2;
    raw[4] = (uint8_t)y2;
    /* raw[5]=binning_mode=0, raw[6-7]=reserved */

    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_ROI_BINNING_SET, (uint32_t *)raw, 2);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragSetBiningMode(dmos5031_dev_t *dev, int mode)
{
    if (mode < 0 || mode > 2) {
        ESP_LOGE(TAG, "DragSetBiningMode: invalid mode %d (0-2)", mode);
        return -1;
    }
    uint8_t raw[8] = {0};
    raw[0] = 2;                    /* roi_binning_en = Binning set     */
    raw[5] = (uint8_t)mode;

    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_ROI_BINNING_SET, (uint32_t *)raw, 2);
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  IR Limit                                                           */
/* ================================================================== */

int DragSetIRLimit(dmos5031_dev_t *dev, int limit)
{
    if (limit < 0 || limit > 4095) {
        ESP_LOGE(TAG, "DragSetIRLimit: invalid limit %d (0-4095)", limit);
        return -1;
    }
    uint32_t val = (uint32_t)limit;
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_IR_LIMIT_SET, &val, 1);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragGetIRLimit(dmos5031_dev_t *dev, int *limit)
{
    uint32_t val = 0;
    esp_err_t ret = dmos5031_get_cmd_value(dev, DRAG_CMD_IR_LIMIT_GET, &val, 1);
    if (ret == ESP_OK && limit) *limit = (int)val;
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  AP Sync                                                            */
/* ================================================================== */

int DragSetAPSync(dmos5031_dev_t *dev, int enable)
{
    uint32_t val = (uint32_t)(enable ? 1 : 0);
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_AP_SYNC_SWITCH, &val, 1);
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  Anti-MMI (Multi-Module Interference)                                */
/* ================================================================== */

int DragSetAntiMMI(dmos5031_dev_t *dev, const dragonfly_antimmi_t *cfg)
{
    if (!cfg) return -1;
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_ANTIMMI_SET, (const uint32_t *)cfg, 1);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragGetAntiMMI(dmos5031_dev_t *dev, dragonfly_antimmi_t *cfg)
{
    if (!cfg) return -1;
    uint32_t val = 0;
    esp_err_t ret = dmos5031_get_cmd_value(dev, DRAG_CMD_ANTIMMI_GET, &val, 1);
    if (ret == ESP_OK) memcpy(cfg, &val, sizeof(*cfg));
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  Lens Calibration                                                   */
/* ================================================================== */

int DragGetLensCoeff(dmos5031_dev_t *dev, dragonfly_lens_coeff_t *cali)
{
    if (!cali) return -1;

    /* LensCoeff = 44 bytes = 11 uint32_t words */
    uint32_t buf[12] = {0};
    esp_err_t ret = dmos5031_get_cmd_value(dev, DRAG_CMD_SENSOR_INFO_GET, buf, 11);
    if (ret != ESP_OK) return -1;

    /* The response overlays the LensCoeff struct directly */
    dragonfly_lens_coeff_t *p = (dragonfly_lens_coeff_t *)buf;
    cali->cali_mode = p->cali_mode;
    cali->fx       = p->fx;
    cali->fy       = p->fy;
    cali->u0       = p->u0;
    cali->v0       = p->v0;
    cali->k1       = p->k1;
    cali->k2       = p->k2;
    cali->k3       = p->k3;
    cali->k4_p1    = p->k4_p1;
    cali->k5_p2    = p->k5_p2;
    cali->skew     = p->skew;
    return 0;
}

static float s5p27_to_float(uint32_t v)
{
    /* s5p27: bit 31 = sign, rest = magnitude.  divisor = 2^27 = 134217728.0 */
    if (v & 0x80000000UL) {
        return -(float)((~v + 1UL) & 0x7FFFFFFFUL) / 134217728.0f;
    }
    return (float)v / 134217728.0f;
}

int DragGetCalib(dmos5031_dev_t *dev, float *calib_buf)
{
    if (!calib_buf) return -1;

    dragonfly_lens_coeff_t cali;
    if (DragGetLensCoeff(dev, &cali) != 0) {
        ESP_LOGE(TAG, "DragGetCalib: lens coeff read failed");
        return -1;
    }

    /* u14p18: divisor = 2^18 = 262144.0 */
    const float u14p18 = 1.0f / 262144.0f;
    calib_buf[0] = (float)cali.fx * u14p18;       /* fx   */
    calib_buf[1] = (float)cali.fy * u14p18;       /* fy   */
    calib_buf[2] = (float)cali.u0 * u14p18;       /* u0   */
    calib_buf[3] = (float)cali.v0 * u14p18;       /* v0   */
    calib_buf[4] = s5p27_to_float(cali.k1);       /* k1   */
    calib_buf[5] = s5p27_to_float(cali.k2);       /* k2   */
    calib_buf[6] = s5p27_to_float(cali.k3);       /* k3   */
    calib_buf[7] = s5p27_to_float(cali.k4_p1);    /* k4/p1*/
    calib_buf[8] = s5p27_to_float(cali.k5_p2);    /* k5/p2*/
    /* s8p24: divisor = 2^24 = 16777216.0 */
    if (cali.skew & 0x80000000UL)
        calib_buf[9] = -(float)((~cali.skew + 1UL) & 0x7FFFFFFFUL) / 16777216.0f;
    else
        calib_buf[9] = (float)cali.skew / 16777216.0f;
    return 0;
}

/* ================================================================== */
/*  AE Window                                                          */
/* ================================================================== */

int DragSetAEWindow(dmos5031_dev_t *dev, const dragonfly_ae_window_t *ae_roi)
{
    if (!ae_roi) return -1;
    /* DragonflyAEWindow = {u16 x0,y0,x1,y1} = 8 bytes = 2 uint32_t */
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_AE_WINDOW_SET, (const uint32_t *)ae_roi, 2);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragGetAEWindow(dmos5031_dev_t *dev, dragonfly_ae_window_t *ae_roi)
{
    if (!ae_roi) return -1;
    uint32_t buf[2] = {0};
    esp_err_t ret = dmos5031_get_cmd_value(dev, DRAG_CMD_AE_WINDOW_GET, buf, 2);
    if (ret == ESP_OK) memcpy(ae_roi, buf, sizeof(*ae_roi));
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  Flash / Solidify config                                            */
/* ================================================================== */

int DragSetISPSolidifiedFlash(dmos5031_dev_t *dev)
{
    /* No payload — just the command */
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_ISP_INIT_SOLIDIFY, NULL, 0);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragErasureISPSolidifiedConfig(dmos5031_dev_t *dev)
{
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_SOLIDIFY_ERASURE, NULL, 0);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragGetISPSolidifiedConfig(dmos5031_dev_t *dev, dmos5031_isp_init_set_t *cfg)
{
    if (!cfg) return -1;
    /* ISPInitSet = ~11 bytes → 3 uint32_t words */
    uint32_t buf[4] = {0};
    esp_err_t ret = dmos5031_get_cmd_value(dev, DRAG_CMD_SOLIDIFY_CONFIG_GET, buf, 3);
    if (ret == ESP_OK) memcpy(cfg, buf, sizeof(*cfg));
    return (ret == ESP_OK) ? 0 : -1;
}

/* ================================================================== */
/*  Obstacle Detection                                                 */
/* ================================================================== */

int DragSetObstacleDetectTrigger(dmos5031_dev_t *dev, const dragonfly_obst_detect_t *cfg)
{
    if (!cfg) return -1;
    esp_err_t ret = dmos5031_send_cmd(dev, DRAG_CMD_OBST_DETECT_TRIGGER, (const uint32_t *)cfg, 1);
    return (ret == ESP_OK) ? 0 : -1;
}

int DragGetObstacleDetectRefPlaneResult(dmos5031_dev_t *dev, dragonfly_plane_result_t *result)
{
    if (!result) return -1;
    uint32_t buf[2] = {0};
    esp_err_t ret = dmos5031_get_cmd_value(dev, DRAG_CMD_OBST_REF_PLANE_GET, buf, 2);
    if (ret == ESP_OK) memcpy(result, buf, sizeof(*result));
    return (ret == ESP_OK) ? 0 : -1;
}
