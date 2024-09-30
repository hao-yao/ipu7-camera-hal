/*
 * Copyright (C) 2015-2024 Intel Corporation.
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

#define LOG_TAG AiqUnit

#include <map>
#include <string>
#include <memory>

#include "iutils/Errors.h"
#include "iutils/CameraLog.h"
#include "CameraContext.h"
#include "PlatformData.h"

#include "AiqUnit.h"

namespace icamera {

AiqUnit::AiqUnit(int cameraId, SensorHwCtrl *sensorHw, LensHw *lensHw) :
    mCameraId(cameraId),
    mAiqUnitState(AIQ_UNIT_NOT_INIT),
#ifndef PAC_ENABLE
#endif
    mCcaInitialized(false) {
    mAiqEngine = new AiqEngine(cameraId, sensorHw, lensHw);

#ifndef PAC_ENABLE
#endif
}

AiqUnit::~AiqUnit() {
    if (mAiqUnitState == AIQ_UNIT_START) {
        stop();
    }
    if (mAiqUnitState == AIQ_UNIT_INIT) {
        deinit();
    }

#ifndef PAC_ENABLE
#endif
    delete mAiqEngine;
}

int AiqUnit::init() {
    AutoMutex l(mAiqUnitLock);
    LOG1("<id%d>@%s", mCameraId, __func__);

    if (mAiqUnitState == AIQ_UNIT_NOT_INIT) {
        int ret = mAiqEngine->init();
        if (ret != OK) {
            mAiqEngine->deinit();
            return ret;
        }
    }

    mAiqUnitState = AIQ_UNIT_INIT;

    return OK;
}

int AiqUnit::deinit() {
    AutoMutex l(mAiqUnitLock);
    LOG1("<id%d>@%s", mCameraId, __func__);

    mAiqEngine->deinit();

    deinitIntelCcaHandle();
    mAiqUnitState = AIQ_UNIT_NOT_INIT;

    return OK;
}

int AiqUnit::configure(const stream_config_t *streamList) {
    CheckAndLogError(streamList == nullptr, BAD_VALUE, "streamList is nullptr");

    AutoMutex l(mAiqUnitLock);
    LOG1("<id%d>@%s", mCameraId, __func__);

    if (mAiqUnitState != AIQ_UNIT_INIT && mAiqUnitState != AIQ_UNIT_STOP) {
        LOGW("%s: configure in wrong state: %d", __func__, mAiqUnitState);
        return BAD_VALUE;
    }

    std::vector<ConfigMode> configModes;
    PlatformData::getConfigModesByOperationMode(mCameraId, streamList->operation_mode,
                                                configModes);
    int ret = initIntelCcaHandle(configModes);
    CheckAndLogError(ret < 0, BAD_VALUE, "@%s failed to create intel cca handle", __func__);

    ret = mAiqEngine->configure();
    CheckAndLogError(ret != OK, ret, "configure AIQ engine error: %d", ret);

    mAiqUnitState = AIQ_UNIT_CONFIGURED;
    return OK;
}

int AiqUnit::initIntelCcaHandle(const std::vector<ConfigMode> &configModes) {
    if (mCcaInitialized) return OK;

    LOG1("<id%d>@%s", mCameraId, __func__);
    mTuningModes.clear();
    for (auto &cfg : configModes) {
        TuningMode tuningMode;
        int ret = PlatformData::getTuningModeByConfigMode(mCameraId, cfg, tuningMode);
        CheckAndLogError(ret != OK, ret, "%s: Failed to get tuningMode, cfg: %d", __func__, cfg);

        PERF_CAMERA_ATRACE_PARAM1_IMAGING("intelCca->init", 1);

        // Initialize cca_cpf data
        ia_binary_data cpfData;
        cca::cca_init_params params = {};
        ret = PlatformData::getCpf(mCameraId, tuningMode, &cpfData);
        if (ret == OK && cpfData.data) {
            CheckAndLogError(cpfData.size > cca::MAX_CPF_LEN, UNKNOWN_ERROR,
                       "%s, AIQB buffer is too small cpfData:%d > MAX_CPF_LEN:%d",
                       __func__, cpfData.size, cca::MAX_CPF_LEN);
            MEMCPY_S(params.aiq_cpf.buf, cca::MAX_CPF_LEN, cpfData.data, cpfData.size);
            params.aiq_cpf.size = cpfData.size;
        }

        // Initialize cca_nvm data
        ia_binary_data* nvmData = PlatformData::getNvm(mCameraId);
        if (nvmData) {
            CheckAndLogError(nvmData->size > cca::MAX_NVM_LEN,  UNKNOWN_ERROR,
                       "%s, NVM buffer is too small: nvmData:%d  MAX_NVM_LEN:%d",
                       __func__, nvmData->size, cca::MAX_NVM_LEN);
            MEMCPY_S(params.aiq_nvm.buf, cca::MAX_NVM_LEN, nvmData->data, nvmData->size);
            params.aiq_nvm.size = nvmData->size;
        }

        // Initialize cca_aiqd data
        ia_binary_data* aiqdData = PlatformData::getAiqd(mCameraId, tuningMode);
        if (aiqdData) {
            CheckAndLogError(aiqdData->size > cca::MAX_AIQD_LEN,  UNKNOWN_ERROR,
                       "%s, AIQD buffer is too small aiqdData:%d > MAX_AIQD_LEN:%d",
                       __func__, aiqdData->size, cca::MAX_AIQD_LEN);
            MEMCPY_S(params.aiq_aiqd.buf, cca::MAX_AIQD_LEN, aiqdData->data, aiqdData->size);
            params.aiq_aiqd.size = aiqdData->size;
        }

        SensorFrameParams sensorParam = {};
        ret = PlatformData::calculateFrameParams(mCameraId, sensorParam);
        CheckAndLogError(ret != OK, ret, "%s: Failed to calculate frame params", __func__);
        AiqUtils::convertToAiqFrameParam(sensorParam, params.frameParams);

        params.frameUse = ia_aiq_frame_use_video;
        params.aiqStorageLen = MAX_SETTING_COUNT;
        // handle AE delay in AiqEngine
        params.aecFrameDelay = 0;

        // Initialize functions which need to be started
        params.bitmap = cca::CCA_MODULE_AE | cca::CCA_MODULE_AWB |
                        cca::CCA_MODULE_PA | cca::CCA_MODULE_SA | cca::CCA_MODULE_GBCE |
                        cca::CCA_MODULE_LARD;
        if (PlatformData::getLensHwType(mCameraId) == LENS_VCM_HW) {
            params.bitmap |= cca::CCA_MODULE_AF;
        }

#ifndef PAC_ENABLE
#endif

        std::shared_ptr<GraphConfig> graphConfig =
            CameraContext::getInstance(mCameraId)->getGraphConfig(cfg);
        if (graphConfig != nullptr) {
            std::vector<int32_t> streamIds;
            graphConfig->graphGetStreamIds(streamIds, false);
            params.aic_stream_ids.count = streamIds.size();
            CheckAndLogError(streamIds.size() > cca::MAX_STREAM_NUM, UNKNOWN_ERROR,
                    "%s, Too many streams: %zu in graph", __func__, streamIds.size());
            for (size_t i = 0; i < streamIds.size(); ++i) {
                params.aic_stream_ids.ids[i] = streamIds[i];
            }
        }

        IntelCca *intelCca = IntelCca::getInstance(mCameraId, tuningMode);
        CheckAndLogError(!intelCca, UNKNOWN_ERROR,
                         "Failed to get cca. mode:%d cameraId:%d", tuningMode, mCameraId);
        ia_err iaErr = intelCca->init(params);
        if (iaErr == ia_err_none) {
            mTuningModes.push_back(tuningMode);
        } else {
            LOGE("%s, init IntelCca fails. mode:%d cameraId:%d", __func__, tuningMode, mCameraId);
            IntelCca::releaseInstance(mCameraId, tuningMode);
            return UNKNOWN_ERROR;
        }

        ret = PlatformData::initMakernote(mCameraId, tuningMode);
        CheckAndLogError(ret != OK, UNKNOWN_ERROR, "%s, PlatformData::initMakernote fails",
                         __func__);

        dumpCcaInitParam(params);
    }

    mCcaInitialized = true;
    return OK;
}

void AiqUnit::deinitIntelCcaHandle() {
    if (!mCcaInitialized) return;

    LOG1("<id%d>@%s", mCameraId, __func__);
    for (auto &mode : mTuningModes) {
        IntelCca *intelCca = IntelCca::getInstance(mCameraId, mode);
        CheckAndLogError(!intelCca, VOID_VALUE, "%s, Failed to get cca: mode(%d), cameraId(%d)",
                         __func__, mode, mCameraId);

        if (PlatformData::isAiqdEnabled(mCameraId)) {
            cca::cca_aiqd aiqd = {};
            ia_err iaErr = intelCca->getAiqd(&aiqd);
            if (AiqUtils::convertError(iaErr) == OK) {
                ia_binary_data data = {aiqd.buf, static_cast<unsigned int>(aiqd.size)};
                PlatformData::saveAiqd(mCameraId, mode, data);
            } else {
                LOGW("@%s, failed to get aiqd data, iaErr %d", __func__, iaErr);
            }
        }

        int ret = PlatformData::deinitMakernote(mCameraId, mode);
        if (ret != OK) {
            LOGE("@%s, PlatformData::deinitMakernote fails", __func__);
        }

        intelCca->deinit();
        IntelCca::releaseInstance(mCameraId, mode);
    }

    mCcaInitialized = false;
}

int AiqUnit::start() {
    AutoMutex l(mAiqUnitLock);
    LOG1("<id%d>@%s", mCameraId, __func__);

    if (mAiqUnitState != AIQ_UNIT_CONFIGURED && mAiqUnitState != AIQ_UNIT_STOP) {
        LOGW("%s: configure in wrong state: %d", __func__, mAiqUnitState);
        return BAD_VALUE;
    }

    int ret = mAiqEngine->startEngine();
    if (ret == OK) {
        mAiqUnitState = AIQ_UNIT_START;
    }

    return OK;
}

int AiqUnit::stop() {
    AutoMutex l(mAiqUnitLock);
    LOG1("<id%d>@%s", mCameraId, __func__);

    if (mAiqUnitState == AIQ_UNIT_START) {
        mAiqEngine->stopEngine();
    }

    mAiqUnitState = AIQ_UNIT_STOP;

    return OK;
}

int AiqUnit::run3A(int64_t ccaId, int64_t applyingSeq, int64_t frameNumber, int64_t* effectSeq) {
    AutoMutex l(mAiqUnitLock);
    TRACE_LOG_PROCESS("AiqUnit", "run3A");

    if (mAiqUnitState != AIQ_UNIT_START) {
        LOGW("%s: AIQ is not started: %d", __func__, mAiqUnitState);
        return BAD_VALUE;
    }

    int ret = mAiqEngine->run3A(ccaId, applyingSeq, frameNumber, effectSeq);
    CheckAndLogError(ret != OK, ret, "run 3A failed.");

    return OK;
}

std::vector<EventListener*> AiqUnit::getSofEventListener() {
    AutoMutex l(mAiqUnitLock);
    std::vector<EventListener*> eventListenerList;
    eventListenerList.push_back(mAiqEngine->getSofEventListener());
    return eventListenerList;
}

std::vector<EventListener*> AiqUnit::getStatsEventListener() {
    AutoMutex l(mAiqUnitLock);

    std::vector<EventListener*> eventListenerList;
#ifndef PAC_ENABLE
#endif
    return eventListenerList;
}

void AiqUnit::dumpCcaInitParam(const cca::cca_init_params& params) {
    if (!Log::isLogTagEnabled(GET_FILE_SHIFT(AiqUnit))) return;

    LOG3("bitmap:%x", params.bitmap);
    LOG3("frameUse: %d", params.frameUse);
    LOG3("aecFrameDelay:%d", params.aecFrameDelay);
    LOG3("streamId num:%zu", params.aic_stream_ids.count);

    LOG3("horizontal_crop_offset:%d", params.frameParams.horizontal_crop_offset);
    LOG3("vertical_crop_offset:%d", params.frameParams.vertical_crop_offset);
    LOG3("cropped_image_width:%d", params.frameParams.cropped_image_width);
    LOG3("cropped_image_height:%d", params.frameParams.cropped_image_height);
    LOG3("horizontal_scaling_numerator:%d", params.frameParams.horizontal_scaling_numerator);
    LOG3("horizontal_scaling_denominator:%d", params.frameParams.horizontal_scaling_denominator);
    LOG3("vertical_scaling_numerator:%d", params.frameParams.vertical_scaling_numerator);
    LOG3("vertical_scaling_denominator:%d", params.frameParams.vertical_scaling_denominator);
}

} /* namespace icamera */
