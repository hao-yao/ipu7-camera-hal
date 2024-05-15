/*
 * Copyright (C) 2018-2024 Intel Corporation.
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

#pragma once

#ifdef PAC_ENABLE
#include "ia_aic_types.h"
#else
#include "ia_isp_types.h"
#endif
#include "iutils/Utils.h"
#include "ParamDataType.h"

namespace icamera {

struct IspImageEnhancement{
    char manualSharpness;
    char manualBrightness;
    char manualContrast;
    char manualHue;
    char manualSaturation;
};

struct IspSettings {
    ia_isp_feature_setting nrSetting;
    ia_isp_feature_setting eeSetting;
    ia_isp_effect effects;
    bool videoStabilization;
    IspImageEnhancement manualSettings;
    ia_binary_data* palOverride;
    camera_zoom_region_t zoom;
    camera_mount_type_t sensorMountType;

    IspSettings() { CLEAR(*this); zoom.ratio = 1.0f; }
};

} // namespace icamera

