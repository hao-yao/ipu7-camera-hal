/*
 * Copyright (C) 2021-2022 Intel Corporation.
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

// !!! DO NOT EDIT THIS FILE !!!

#ifndef __MODULE_TAGS_HPP__
#define __MODULE_TAGS_HPP__

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Werror"
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#pragma clang diagnostic ignored "-Wformat-security"
#endif

enum ModuleTags {
      GENERATED_TAGS_3atest = 0,
      GENERATED_TAGS_AiqCore = 1,
      GENERATED_TAGS_AiqEngine = 2,
      GENERATED_TAGS_AiqInitData = 3,
      GENERATED_TAGS_AiqResult = 4,
      GENERATED_TAGS_AiqResultStorage = 5,
      GENERATED_TAGS_AiqSetting = 6,
      GENERATED_TAGS_AiqUnit = 7,
      GENERATED_TAGS_AiqUtils = 8,
      GENERATED_TAGS_BufferAllocator = 9,
      GENERATED_TAGS_BufferQueue = 10,
      GENERATED_TAGS_CASE_3A_CONTROL = 11,
      GENERATED_TAGS_CASE_AIQ = 12,
      GENERATED_TAGS_CASE_API_MULTI_THREAD = 13,
      GENERATED_TAGS_CASE_BUFFER = 14,
      GENERATED_TAGS_CASE_COMMON = 15,
      GENERATED_TAGS_CASE_CPF = 16,
      GENERATED_TAGS_CASE_DEVICE_OPS = 17,
      GENERATED_TAGS_CASE_DUAL = 18,
      GENERATED_TAGS_CASE_GRAPH = 19,
      GENERATED_TAGS_CASE_IQ_EFFECT = 20,
      GENERATED_TAGS_CASE_PARAMETER = 21,
      GENERATED_TAGS_CASE_PER_FRAME = 22,
      GENERATED_TAGS_CASE_STATIC_INFO = 23,
      GENERATED_TAGS_CASE_STREAM_OPS = 24,
      GENERATED_TAGS_CASE_THREAD = 25,
      GENERATED_TAGS_CASE_VIRTUAL_CHANNEL = 26,
      GENERATED_TAGS_CBLayoutUtils = 27,
      GENERATED_TAGS_CBStage = 28,
      GENERATED_TAGS_Camera3AMetadata = 29,
      GENERATED_TAGS_Camera3Buffer = 30,
      GENERATED_TAGS_Camera3Format = 31,
      GENERATED_TAGS_Camera3HAL = 32,
      GENERATED_TAGS_Camera3HALModule = 33,
      GENERATED_TAGS_CameraBuffer = 34,
      GENERATED_TAGS_CameraBufferPool = 35,
      GENERATED_TAGS_CameraContext = 36,
      GENERATED_TAGS_CameraDevice = 37,
      GENERATED_TAGS_CameraDump = 38,
      GENERATED_TAGS_CameraEvent = 39,
      GENERATED_TAGS_CameraHal = 40,
      GENERATED_TAGS_CameraLog = 41,
      GENERATED_TAGS_CameraMetadata = 42,
      GENERATED_TAGS_CameraParserInvoker = 43,
      GENERATED_TAGS_CameraSensorsParser = 44,
      GENERATED_TAGS_CameraShm = 45,
      GENERATED_TAGS_CameraStream = 46,
      GENERATED_TAGS_CaptureUnit = 47,
      GENERATED_TAGS_ColorConverter = 48,
      GENERATED_TAGS_CsiMetaDevice = 49,
      GENERATED_TAGS_Customized3A = 50,
      GENERATED_TAGS_CustomizedAic = 51,
      GENERATED_TAGS_DeviceBase = 52,
      GENERATED_TAGS_Dvs = 53,
      GENERATED_TAGS_EXIFMaker = 54,
      GENERATED_TAGS_EXIFMetaData = 55,
      GENERATED_TAGS_EvcpManager = 56,
      GENERATED_TAGS_ExifCreater = 57,
      GENERATED_TAGS_FaceDetection = 58,
      GENERATED_TAGS_FaceDetectionPVL = 59,
      GENERATED_TAGS_FaceSSD = 60,
      GENERATED_TAGS_FaceStage = 61,
      GENERATED_TAGS_FileSource = 62,
      GENERATED_TAGS_GPUPostProcessor = 63,
      GENERATED_TAGS_GPUPostStage = 64,
      GENERATED_TAGS_GenGfx = 65,
      GENERATED_TAGS_GraphConfig = 66,
      GENERATED_TAGS_GraphConfigManager = 67,
      GENERATED_TAGS_GraphUtils = 68,
      GENERATED_TAGS_HAL_FACE_DETECTION_TEST = 69,
      GENERATED_TAGS_HAL_basic = 70,
      GENERATED_TAGS_HAL_inset_portrait = 71,
      GENERATED_TAGS_HAL_jpeg = 72,
      GENERATED_TAGS_HAL_multi_streams_test = 73,
      GENERATED_TAGS_HAL_rotation_test = 74,
      GENERATED_TAGS_HAL_supported_streams_test = 75,
      GENERATED_TAGS_HAL_yuv = 76,
      GENERATED_TAGS_HalAdaptor = 77,
      GENERATED_TAGS_HalV3Utils = 78,
      GENERATED_TAGS_I3AControlFactory = 79,
      GENERATED_TAGS_ICBMThread = 80,
      GENERATED_TAGS_ICamera = 81,
      GENERATED_TAGS_IFaceDetection = 82,
      GENERATED_TAGS_IPCIntelCca = 83,
      GENERATED_TAGS_IPC_FACE_DETECTION = 84,
      GENERATED_TAGS_IPipeManagerFactory = 85,
      GENERATED_TAGS_IProcessingUnitFactory = 86,
      GENERATED_TAGS_ImageProcessorCore = 87,
      GENERATED_TAGS_ImageScalerCore = 88,
      GENERATED_TAGS_Intel3AParameter = 89,
      GENERATED_TAGS_IntelAEStateMachine = 90,
      GENERATED_TAGS_IntelAFStateMachine = 91,
      GENERATED_TAGS_IntelAWBStateMachine = 92,
      GENERATED_TAGS_IntelAlgoClient = 93,
      GENERATED_TAGS_IntelAlgoCommonClient = 94,
      GENERATED_TAGS_IntelAlgoServer = 95,
      GENERATED_TAGS_IntelCPUAlgoServer = 96,
      GENERATED_TAGS_IntelCca = 97,
      GENERATED_TAGS_IntelCcaClient = 98,
      GENERATED_TAGS_IntelCcaServer = 99,
      GENERATED_TAGS_IntelEVCPClient = 100,
      GENERATED_TAGS_IntelEVCPServer = 101,
      GENERATED_TAGS_IntelEvcp = 102,
      GENERATED_TAGS_IntelFDServer = 103,
      GENERATED_TAGS_IntelFaceDetection = 104,
      GENERATED_TAGS_IntelFaceDetectionClient = 105,
      GENERATED_TAGS_IntelGPUAlgoServer = 106,
      GENERATED_TAGS_IntelICBM = 107,
      GENERATED_TAGS_IntelICBMClient = 108,
      GENERATED_TAGS_IntelICBMServer = 109,
      GENERATED_TAGS_IntelTNR7Stage = 110,
      GENERATED_TAGS_IpuPacAdaptor = 111,
      GENERATED_TAGS_IspControlUtils = 112,
      GENERATED_TAGS_JpegEncoderCore = 113,
      GENERATED_TAGS_JpegMaker = 114,
      GENERATED_TAGS_JsonCommonParser = 115,
      GENERATED_TAGS_JsonParserBase = 116,
      GENERATED_TAGS_LensHw = 117,
      GENERATED_TAGS_LensManager = 118,
      GENERATED_TAGS_LiveTuning = 119,
      GENERATED_TAGS_Ltm = 120,
      GENERATED_TAGS_MANUAL_POST_PROCESSING = 121,
      GENERATED_TAGS_MakerNote = 122,
      GENERATED_TAGS_MediaControl = 123,
      GENERATED_TAGS_MetadataConvert = 124,
      GENERATED_TAGS_MockCamera3HAL = 125,
      GENERATED_TAGS_MockCameraHal = 126,
      GENERATED_TAGS_MockPSysDevice = 127,
      GENERATED_TAGS_MockSysCall = 128,
      GENERATED_TAGS_MsgHandler = 129,
      GENERATED_TAGS_OnePunchIC2 = 130,
      GENERATED_TAGS_OpenSourceGFX = 131,
      GENERATED_TAGS_PSysDevice = 132,
      GENERATED_TAGS_ParameterConvert = 133,
      GENERATED_TAGS_ParameterHelper = 134,
      GENERATED_TAGS_Parameters = 135,
      GENERATED_TAGS_PipeLine = 136,
      GENERATED_TAGS_PipeManager = 137,
      GENERATED_TAGS_PipeManagerStub = 138,
      GENERATED_TAGS_PlatformData = 139,
      GENERATED_TAGS_PnpDebugControl = 140,
      GENERATED_TAGS_PostProcessStage = 141,
      GENERATED_TAGS_PostProcessorBase = 142,
      GENERATED_TAGS_PostProcessorCore = 143,
      GENERATED_TAGS_ProcessingUnit = 144,
      GENERATED_TAGS_RequestManager = 145,
      GENERATED_TAGS_RequestThread = 146,
      GENERATED_TAGS_ResultProcessor = 147,
      GENERATED_TAGS_SWJpegEncoder = 148,
      GENERATED_TAGS_SWPostProcessor = 149,
      GENERATED_TAGS_SchedPolicy = 150,
      GENERATED_TAGS_Scheduler = 151,
      GENERATED_TAGS_SensorHwCtrl = 152,
      GENERATED_TAGS_SensorManager = 153,
      GENERATED_TAGS_SensorOB = 154,
      GENERATED_TAGS_ShareRefer = 155,
      GENERATED_TAGS_SofSource = 156,
      GENERATED_TAGS_SwImageConverter = 157,
      GENERATED_TAGS_SwImageProcessor = 158,
      GENERATED_TAGS_SwPostProcessUnit = 159,
      GENERATED_TAGS_SyncManager = 160,
      GENERATED_TAGS_SysCall = 161,
      GENERATED_TAGS_TCPServer = 162,
      GENERATED_TAGS_Thread = 163,
      GENERATED_TAGS_Trace = 164,
      GENERATED_TAGS_UltraManEvcp = 165,
      GENERATED_TAGS_Utils = 166,
      GENERATED_TAGS_V4l2DeviceFactory = 167,
      GENERATED_TAGS_V4l2_device_cc = 168,
      GENERATED_TAGS_V4l2_subdevice_cc = 169,
      GENERATED_TAGS_V4l2_video_node_cc = 170,
      GENERATED_TAGS_VendorTags = 171,
      GENERATED_TAGS_camera_metadata_tests = 172,
      GENERATED_TAGS_icamera_metadata_base = 173,
      GENERATED_TAGS_metadata_test = 174,
};

#define TAGS_MAX_NUM 175

#endif
// !!! DO NOT EDIT THIS FILE !!!
