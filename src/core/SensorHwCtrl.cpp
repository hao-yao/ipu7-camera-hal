/*
 * Copyright (C) 2015-2021 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG SensorHwCtrl

#include <limits.h>
#include <linux/types.h>
#include <linux/v4l2-controls.h>
// CRL_MODULE_S
#include <linux/crlmodule.h>
// CRL_MODULE_E

#include "PlatformData.h"
#include "SensorHwCtrl.h"
#include "V4l2DeviceFactory.h"
#include "iutils/CameraLog.h"

using std::vector;

namespace icamera {

SensorHwCtrl::SensorHwCtrl(int cameraId, V4L2Subdevice* pixelArraySubdev,
                           V4L2Subdevice* sensorOutputSubdev)
        : mPixelArraySubdev(pixelArraySubdev),
          // CRL_MODULE_S
          mSensorOutputSubdev(sensorOutputSubdev),
          // CRL_MODULE_E
          mCameraId(cameraId),
          mHorzBlank(0),
          mVertBlank(0),
          mCropWidth(0),
          mCropHeight(0),
          // HDR_FEATURE_S
          mWdrMode(0),
          // HDR_FEATURE_E
          mCurFll(0),
          mCalculatingFrameDuration(true) {
    LOG1("<id%d> @%s", mCameraId, __func__);
    // CRL_MODULE_S
    /**
     * Try to call V4L2_CID_LINE_LENGTH_PIXELS, if failed, it means llp can't
     * be read directly from sensor. Then calculate it with HBlank.
     * fll will be in the same case.
     */
    if (mPixelArraySubdev) {
        int llp = 0;
        int status = mPixelArraySubdev->GetControl(V4L2_CID_LINE_LENGTH_PIXELS, &llp);
        if (status == OK) {
            LOG1("%s, some sensors can get llp directly, don't calculate it", __func__);
            mCalculatingFrameDuration = false;
        }
    }
    // CRL_MODULE_E
}

SensorHwCtrl* SensorHwCtrl::createSensorCtrl(int cameraId) {
    if (!PlatformData::isIsysEnabled(cameraId)) {
        return new DummySensor(cameraId);
    }

    std::string subDevName;
    SensorHwCtrl* sensorCtrl = nullptr;
    int ret = PlatformData::getDevNameByType(cameraId, VIDEO_PIXEL_ARRAY, subDevName);
    if (ret == OK) {
        LOG1("%s ArraySubdev camera id:%d dev name:%s", __func__, cameraId, subDevName.c_str());
        V4L2Subdevice* pixelArraySubdev = V4l2DeviceFactory::getSubDev(cameraId, subDevName);

        V4L2Subdevice* pixelOutputSubdev = nullptr;
        // Binner and Scaler subdev only exits in CrlModule driver
        if (PlatformData::isUsingCrlModule(cameraId)) {
            subDevName.clear();
            ret = PlatformData::getDevNameByType(cameraId, VIDEO_PIXEL_SCALER, subDevName);
            if (ret == OK) {
                LOG1("%s ScalerSubdev camera id:%d dev name:%s", __func__, cameraId,
                     subDevName.c_str());
                pixelOutputSubdev = V4l2DeviceFactory::getSubDev(cameraId, subDevName);
            } else {
                subDevName.clear();
                ret = PlatformData::getDevNameByType(cameraId, VIDEO_PIXEL_BINNER, subDevName);
                if (ret == OK) {
                    LOG1("%s BinnerSubdev camera id:%d dev name:%s", __func__, cameraId,
                         subDevName.c_str());
                    pixelOutputSubdev = V4l2DeviceFactory::getSubDev(cameraId, subDevName);
                }
            }
        }

        sensorCtrl = new SensorHwCtrl(cameraId, pixelArraySubdev, pixelOutputSubdev);
    } else {
        LOG1("%s create a dummy sensor ctrl for camera id:%d", __func__, cameraId);
        sensorCtrl = new DummySensor(cameraId);
    }
    return sensorCtrl;
}

// CRL_MODULE_S
int SensorHwCtrl::configure() {
    return OK;
}
// CRL_MODULE_E

int SensorHwCtrl::getActivePixelArraySize(int& width, int& height, int& pixelCode) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");

    int status = mPixelArraySubdev->GetPadFormat(0, &width, &height, &pixelCode);
    mCropWidth = width;
    mCropHeight = height;

    LOG2("@%s, width:%d, height:%d, status:%d", __func__, width, height, status);
    return status;
}

int SensorHwCtrl::getPixelRate(int& pixelRate) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");

    int ret = mPixelArraySubdev->GetControl(V4L2_CID_PIXEL_RATE, &pixelRate);

    LOG2("@%s, pixelRate:%d, ret:%d", __func__, pixelRate, ret);

    return ret;
}

int SensorHwCtrl::setTestPatternMode(int32_t testPatternMode) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");

    LOG2("@%s, testPatternMode: %d", __func__, testPatternMode);
    return mPixelArraySubdev->SetControl(V4L2_CID_TEST_PATTERN, testPatternMode);
}

int SensorHwCtrl::setExposure(const vector<int>& coarseExposures,
                              const vector<int>& fineExposures) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");
    CheckAndLogError((coarseExposures.empty() || fineExposures.empty()), BAD_VALUE,
                     "No exposure data!");

    // CRL_MODULE_S
    if (coarseExposures.size() > 1) {
        if (PlatformData::getSensorExposureType(mCameraId) == SENSOR_MULTI_EXPOSURES) {
            return setMultiExposures(coarseExposures, fineExposures);
        } else if (PlatformData::getSensorExposureType(mCameraId) ==
                   SENSOR_DUAL_EXPOSURES_DCG_AND_VS) {
            return setDualExposuresDCGAndVS(coarseExposures, fineExposures);
        }
    }
    // CRL_MODULE_E

    LOG2("%s coarseExposure=%d fineExposure=%d", __func__, coarseExposures[0], fineExposures[0]);
    LOG2("SENSORCTRLINFO: exposure_value=%d", coarseExposures[0]);
    return mPixelArraySubdev->SetControl(V4L2_CID_EXPOSURE, coarseExposures[0]);
}

// CRL_MODULE_S
int SensorHwCtrl::setMultiExposures(const vector<int>& coarseExposures,
                                    const vector<int>& fineExposures) {
    int status = BAD_VALUE;
    int shortExp = coarseExposures[0];
    int longExp = coarseExposures[1];

    if (coarseExposures.size() > 2) {
        LOG2("coarseExposure[0]=%d fineExposure[0]=%d", coarseExposures[0], fineExposures[0]);
        // The first exposure is very short exposure if larger than 2 exposures.
        status = mPixelArraySubdev->SetControl(CRL_CID_EXPOSURE_SHS2, coarseExposures[0]);
        CheckAndLogError(status != OK, status, "failed to set exposure SHS2 %d.",
                         coarseExposures[0]);

        shortExp = coarseExposures[1];
        longExp = coarseExposures[2];

        LOG2("SENSORCTRLINFO: exposure_long=%d", coarseExposures[2]);   // long
        LOG2("SENSORCTRLINFO: exposure_med=%d", coarseExposures[1]);    // short
        LOG2("SENSORCTRLINFO: exposure_short=%d", coarseExposures[0]);  // very short
    }

    LOG2("shortExp=%d longExp=%d", shortExp, longExp);
    status = mPixelArraySubdev->SetControl(CRL_CID_EXPOSURE_SHS1, shortExp);
    CheckAndLogError(status != OK, status, "failed to set exposure SHS1 %d.", shortExp);

    status = mPixelArraySubdev->SetControl(V4L2_CID_EXPOSURE, longExp);
    CheckAndLogError(status != OK, status, "failed to set long exposure %d.", longExp);
    LOG2("SENSORCTRLINFO: exposure_value=%d", longExp);

    return status;
}

int SensorHwCtrl::setDualExposuresDCGAndVS(const vector<int>& coarseExposures,
                                           const vector<int>& fineExposures) {
    int status = BAD_VALUE;
    int longExp = coarseExposures[1];

    if (coarseExposures.size() > 2) {
        LOG2("coarseExposure[0]=%d fineExposure[0]=%d", coarseExposures[0], fineExposures[0]);
        // The first exposure is very short exposure for DCG + VS case.
        status = mPixelArraySubdev->SetControl(CRL_CID_EXPOSURE_SHS1, coarseExposures[0]);
        CheckAndLogError(status != OK, status, "failed to set exposure SHS1 %d.",
                         coarseExposures[0]);

        longExp = coarseExposures[2];
        LOG2("SENSORCTRLINFO: exposure_long=%d", coarseExposures[2]);  // long
    }

    status = mPixelArraySubdev->SetControl(V4L2_CID_EXPOSURE, longExp);
    CheckAndLogError(status != OK, status, "failed to set long exposure %d.", longExp);
    LOG2("SENSORCTRLINFO: exposure_value=%d", longExp);

    return status;
}
// CRL_MODULE_E

int SensorHwCtrl::setAnalogGains(const vector<int>& analogGains) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");
    CheckAndLogError(analogGains.empty(), BAD_VALUE, "No analog gain data!");

    // CRL_MODULE_S
    if (analogGains.size() > 1) {
        if (PlatformData::getSensorGainType(mCameraId) == SENSOR_MULTI_DG_AND_CONVERTION_AG) {
            return setConversionGain(analogGains);
        } else if (PlatformData::getSensorGainType(mCameraId) == SENSOR_MULTI_DG_AND_DIRECT_AG) {
            LOG2("sensor multi conversion gain");
            return setMultiAnalogGain(analogGains);
        }
    }
    // CRL_MODULE_E

    LOG2("%s analogGain=%d", __func__, analogGains[0]);
    return mPixelArraySubdev->SetControl(V4L2_CID_ANALOGUE_GAIN, analogGains[0]);
}

int SensorHwCtrl::setDigitalGains(const vector<int>& digitalGains) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");
    CheckAndLogError(digitalGains.empty(), BAD_VALUE, "No digital gain data!");

    // CRL_MODULE_S
    if (digitalGains.size() > 1) {
        if (PlatformData::getSensorGainType(mCameraId) == SENSOR_MULTI_DG_AND_CONVERTION_AG) {
            return setMultiDigitalGain(digitalGains);
        } else if (PlatformData::getSensorGainType(mCameraId) == SENSOR_MULTI_DG_AND_DIRECT_AG) {
            LOG2("sensor multi conversion gain");
            return setMultiDigitalGain(digitalGains);
        }
    }

    if (mWdrMode && PlatformData::getSensorGainType(mCameraId) == ISP_DG_AND_SENSOR_DIRECT_AG) {
        LOG2("%s: WDR mode, skip sensor DG, all digital gain is passed to ISP", __func__);
    } else if (PlatformData::isUsingSensorDigitalGain(mCameraId)) {
        if (mPixelArraySubdev->SetControl(V4L2_CID_GAIN, digitalGains[0]) != OK) {
            LOGW("set digital gain failed");
        }
    }
    // CRL_MODULE_E

    LOG2("%s digitalGain=%d", __func__, digitalGains[0]);
    return mPixelArraySubdev->SetControl(V4L2_CID_DIGITAL_GAIN, digitalGains[0]);
}

// CRL_MODULE_S
int SensorHwCtrl::setMultiDigitalGain(const vector<int>& digitalGains) {
    int status = BAD_VALUE;
    int shortDg = digitalGains[0];
    int longDg = digitalGains[1];

    if (digitalGains.size() > 2) {
        LOG2("digitalGains[0]=%d", digitalGains[0]);
        status = mPixelArraySubdev->SetControl(CRL_CID_DIGITAL_GAIN_VS, digitalGains[0]);
        CheckAndLogError(status != OK, status, "failed to set very short DG %d.", digitalGains[0]);

        shortDg = digitalGains[1];
        longDg = digitalGains[2];
    }

    LOG2("shortDg=%d longDg=%d", shortDg, longDg);
    status = mPixelArraySubdev->SetControl(CRL_CID_DIGITAL_GAIN_S, shortDg);
    CheckAndLogError(status != OK, status, "failed to set short DG %d.", shortDg);

    status = mPixelArraySubdev->SetControl(V4L2_CID_GAIN, longDg);
    CheckAndLogError(status != OK, status, "failed to set long DG %d.", longDg);

    return status;
}

int SensorHwCtrl::setMultiAnalogGain(const vector<int>& analogGains) {
    int status = BAD_VALUE;
    int shortAg = analogGains[0];
    int longAg = analogGains[1];

    if (analogGains.size() > 2) {
        LOG2("VS AG %d", analogGains[0]);
        int status = mPixelArraySubdev->SetControl(CRL_CID_ANALOG_GAIN_VS, analogGains[0]);
        CheckAndLogError(status != OK, status, "failed to set VS AG %d", analogGains[0]);

        shortAg = analogGains[1];
        longAg = analogGains[2];

        LOG2("SENSORCTRLINFO: gain_long=%d", analogGains[2]);   // long
        LOG2("SENSORCTRLINFO: gain_med=%d", analogGains[1]);    // short
        LOG2("SENSORCTRLINFO: gain_short=%d", analogGains[0]);  // very short
    }

    LOG2("shortAg=%d longAg=%d", shortAg, longAg);
    status = mPixelArraySubdev->SetControl(CRL_CID_ANALOG_GAIN_S, shortAg);
    CheckAndLogError(status != OK, status, "failed to set short AG %d.", shortAg);

    status = mPixelArraySubdev->SetControl(V4L2_CID_ANALOGUE_GAIN, longAg);
    CheckAndLogError(status != OK, status, "failed to set long AG %d.", longAg);

    return status;
}

int SensorHwCtrl::setConversionGain(const vector<int>& analogGains) {
    CheckAndLogError(analogGains.size() < 2, BAD_VALUE, "Gain data error!");

    /* [0, 1] bits are long AG, [2, 3] bits are short AG, [4, 5] bits are very short AG.
       [6] bit is long conversion gain, [7] bit is very short conversion gain.
       Long AG:       0x0X0000XX
       Short AG:      0x0000XX00
       Very Short AG: 0xX0XX0000 */
    int value = analogGains[0] | analogGains[1] | analogGains[2];
    LOG2("very short AG %d, short AG %d, long AG %d, conversion value %d", analogGains[0],
         analogGains[1], analogGains[2], value);

    int status = mPixelArraySubdev->SetControl(V4L2_CID_ANALOGUE_GAIN, value);
    CheckAndLogError(status != OK, status, "failed to set AG %d", value);

    return OK;
}
// CRL_MODULE_E

int SensorHwCtrl::setLineLengthPixels(int llp) {
    int status = OK;
    LOG2("@%s, llp:%d", __func__, llp);

    if (mCalculatingFrameDuration) {
        int horzBlank = llp - mCropWidth;
        if (mHorzBlank != horzBlank) {
            status = mPixelArraySubdev->SetControl(V4L2_CID_HBLANK, horzBlank);
        }
        // CRL_MODULE_S
    } else {
        status = mPixelArraySubdev->SetControl(V4L2_CID_LINE_LENGTH_PIXELS, llp);
        // CRL_MODULE_E
    }

    CheckAndLogError(status != OK, status, "failed to set llp.");

    mHorzBlank = llp - mCropWidth;
    return status;
}

int SensorHwCtrl::setFrameLengthLines(int fll) {
    int status = OK;
    LOG2("@%s, fll:%d", __func__, fll);

    if (mCalculatingFrameDuration) {
        int vertBlank = fll - mCropHeight;
        if (mVertBlank != vertBlank) {
            status = mPixelArraySubdev->SetControl(V4L2_CID_VBLANK, vertBlank);
        }
        // CRL_MODULE_S
    } else {
        status = mPixelArraySubdev->SetControl(V4L2_CID_FRAME_LENGTH_LINES, fll);
        // CRL_MODULE_E
    }

    mCurFll = fll;

    CheckAndLogError(status != OK, status, "failed to set fll.");

    mVertBlank = fll - mCropHeight;
    return status;
}

int SensorHwCtrl::setFrameDuration(int llp, int fll) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");

    int status = OK;
    LOG2("@%s, llp:%d, fll:%d", __func__, llp, fll);

    /* only set them to driver when llp or fll is not 0 */
    if (llp) {
        status = setLineLengthPixels(llp);
    }

    if (fll) {
        status |= setFrameLengthLines(fll);
    }

    return status;
}

int SensorHwCtrl::getLineLengthPixels(int& llp) {
    int status = OK;

    if (mCalculatingFrameDuration) {
        int horzBlank = 0;
        status = mPixelArraySubdev->GetControl(V4L2_CID_HBLANK, &horzBlank);
        if (status == OK) {
            mHorzBlank = horzBlank;
            llp = horzBlank + mCropWidth;
        }
        // CRL_MODULE_S
    } else {
        status = mPixelArraySubdev->GetControl(V4L2_CID_LINE_LENGTH_PIXELS, &llp);
        if (status == OK) {
            mHorzBlank = llp - mCropWidth;
        }
        // CRL_MODULE_E
    }

    LOG2("@%s, llp:%d", __func__, llp);
    CheckAndLogError(status != OK, status, "failed to get llp.");

    return status;
}

int SensorHwCtrl::getFrameLengthLines(int& fll) {
    int status = OK;

    if (mCalculatingFrameDuration) {
        int vertBlank = 0;
        status = mPixelArraySubdev->GetControl(V4L2_CID_VBLANK, &vertBlank);
        if (status == OK) {
            mVertBlank = vertBlank;
            fll = vertBlank + mCropHeight;
        }
        // CRL_MODULE_S
    } else {
        status = mPixelArraySubdev->GetControl(V4L2_CID_FRAME_LENGTH_LINES, &fll);
        if (status == OK) {
            mVertBlank = fll - mCropHeight;
        }
        // CRL_MODULE_E
    }

    LOG2("@%s, fll:%d", __func__, fll);
    CheckAndLogError(status != OK, status, "failed to get fll.");

    return status;
}

int SensorHwCtrl::getFrameDuration(int& llp, int& fll) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");

    int status = getLineLengthPixels(llp);

    status |= getFrameLengthLines(fll);
    LOG2("@%s, llp:%d, fll:%d", __func__, llp, fll);

    return status;
}

int SensorHwCtrl::getVBlank(int& vblank) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    vblank = mVertBlank;
    LOG2("@%s, vblank:%d", __func__, vblank);

    return OK;
}

/**
 * get exposure range value from sensor driver
 *
 * \param[OUT] coarse_exposure: exposure min value
 * \param[OUT] fine_exposure: exposure max value
 * \param[OUT] exposure_step: step of exposure
 * V4L2 does not support FINE_EXPOSURE setting
 *
 * \return OK if successfully.
 */
int SensorHwCtrl::getExposureRange(int& exposureMin, int& exposureMax, int& exposureStep) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");

    v4l2_queryctrl exposure = {};
    exposure.id = V4L2_CID_EXPOSURE;
    int status = mPixelArraySubdev->QueryControl(&exposure);
    CheckAndLogError(status != OK, status, "Couldn't get exposure Range status:%d", status);

    exposureMin = exposure.minimum;
    exposureMax = exposure.maximum;
    exposureStep = exposure.step;
    LOG2("@%s, exposureMin:%d, exposureMax:%d, exposureStep:%d", __func__, exposureMin, exposureMax,
         exposureStep);

    return status;
}

// HDR_FEATURE_S
int SensorHwCtrl::setWdrMode(int mode) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mSensorOutputSubdev, NO_INIT, "sensor output sub device is not set");

    LOG2("%s WDR Mode=%d", __func__, mode);
    int ret = OK;

    mWdrMode = mode;

    if (PlatformData::getSensorExposureType(mCameraId) != SENSOR_RELATIVE_MULTI_EXPOSURES &&
        PlatformData::getSensorExposureType(mCameraId) != SENSOR_DUAL_EXPOSURES_DCG_AND_VS) {
        LOG2("%s: set WDR mode", __func__);
        ret = mSensorOutputSubdev->SetControl(V4L2_CID_WDR_MODE, mode);
    }

    return ret;
}

int SensorHwCtrl::setAWB(float r_per_g, float b_per_g) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mPixelArraySubdev, NO_INIT, "pixel array sub device is not set");

    LOG2("%s set AWB r_per_g=%f, b_per_g=%f", __func__, r_per_g, b_per_g);

    int ret = mPixelArraySubdev->SetControl(V4L2_CID_RED_BALANCE, static_cast<int>(r_per_g * 256));
    ret |= mPixelArraySubdev->SetControl(V4L2_CID_BLUE_BALANCE, static_cast<int>(b_per_g * 256));

    return ret;
}
// HDR_FEATURE_E

// CRL_MODULE_S
int SensorHwCtrl::setFrameRate(float fps) {
    HAL_TRACE_CALL(CAMERA_DEBUG_LOG_LEVEL2);
    CheckAndLogError(!mSensorOutputSubdev, NO_INIT, "sensor output sub device is not set");

    struct v4l2_queryctrl query;
    CLEAR(query);
    query.id = V4L2_CID_LINK_FREQ;
    int status = mSensorOutputSubdev->QueryControl(&query);
    CheckAndLogError(status != OK, status, "Couldn't get V4L2_CID_LINK_FREQ, status:%d", status);

    LOG2("@%s, query V4L2_CID_LINK_FREQ:, default_value:%d, maximum:%d, minimum:%d, step:%d",
         __func__, query.default_value, query.maximum, query.minimum, query.step);

    int mode = 0;
    if (query.maximum == query.minimum) {
        mode = query.default_value;
    } else {
        /***********************************************************************************
         * WA: This heavily depends on sensor driver implementation, need to find a graceful
         * solution.
         * imx185:
         * When fps larger than 30, should switch to high speed mode, currently only
         * 0, 1, 2 are available. 0 means 720p 30fps, 1 means 2M 30fps, and 2 means 2M 60fps.
         * imx290:
         * 0 and 1 available, for 30 and higher FPS.
         ***********************************************************************************/
        mode = (fps > 30) ? query.maximum : (query.maximum - 1);
    }
    LOG2("@%s, set V4L2_CID_LINK_FREQ to %d, fps %f", __func__, mode, fps);
    return mSensorOutputSubdev->SetControl(V4L2_CID_LINK_FREQ, mode);
}
// CRL_MODULE_E

}  // namespace icamera
