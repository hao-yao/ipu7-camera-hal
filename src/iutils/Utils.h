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

#ifndef UTILS_H
#define UTILS_H

#include <string.h>
#include <string>
#include <vector>

#include <linux/videodev2.h>
#include <v4l2_device.h>
#include "CameraTypes.h"

namespace icamera {

typedef int64_t nsecs_t;

#define ALIGN(val, alignment) (((static_cast<uintptr_t>(val)) +\
        (static_cast<uintptr_t>(alignment)) - 1U) &\
        ~((static_cast<uintptr_t>(alignment)) - 1U))
#define ALIGN_64(val) ALIGN(val, 64)
#define ALIGN_32(val) ALIGN(val, 32)
#define ALIGN_16(val) ALIGN(val, 16)
#define ALIGN_8(val) ALIGN(val, 8)

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define CLEAR(x) (void)memset(&(x), 0, sizeof(x))

// macro CLIP is used to clip the Number value to between the Min and Max
#define CLIP(Number, Max, Min) ((Number) > (Max) ? (Max) : ((Number) < (Min) ? (Min) : (Number)))

#ifndef UNUSED
#define UNUSED(param) (void)(param)
#endif

/**
 * Align to page boundary
 * \ingroup ia_tools
 */
#ifndef PAGE_ALIGN
#define PAGE_SIZE_U (getpagesize())
#define PAGE_ALIGN(x) ALIGN(x, PAGE_SIZE_U)
#endif

/* Integer round-up division of a with b */
#define CEIL_DIV(a, b) ((b) ? (((a) + (b)-1) / (b)) : 0)
/* Align a to the upper multiple of b */
#define CEIL_MUL(a, b) CEIL_DIV(a, b) * (b)

#define SINGLE_FIELD(field) \
    ((field == V4L2_FIELD_TOP) || (field == V4L2_FIELD_BOTTOM) || (field == V4L2_FIELD_ALTERNATE))
/**
 * Use to check input parameter and if failed, return err_code and print error message
 */
#define VOID_VALUE
#define CheckAndLogError(condition, err_code, err_msg, args...) \
    do {                                                        \
        if (condition) {                                        \
            LOGE(err_msg, ##args);                              \
            return err_code;                                    \
        }                                                       \
    } while (0)

#define CheckAndClean(condition, err_code, clean, err_msg, args...) \
    do {                                                            \
        if (condition) {                                            \
            LOGE(err_msg, ##args);                                  \
            clean;                                                  \
            return err_code;                                        \
        }                                                           \
    } while (0)

/**
 * Use to check input parameter and if failed, return err_code and print warning message,
 * this should be used for non-vital error checking.
 */
#define CheckWarning(condition, err_code, err_msg, args...) \
    do {                                                    \
        if (condition) {                                    \
            LOGW(err_msg, ##args);                          \
            return err_code;                                \
        }                                                   \
    } while (0)

// As above but no return.
#define CheckWarningNoReturn(condition, err_msg, args...) \
    do {                                                  \
        if (condition) {                                  \
            LOGW(err_msg, ##args);                        \
        }                                                 \
    } while (0)

// macro delete array and set it to null
#define DELETE_ARRAY_AND_NULLIFY(var) \
    do {                              \
        delete[](var);                \
        var = nullptr;                \
    } while (0)

/**
 *  \macro TIMEVAL2NSECS
 *  Convert timeval struct to value in nanoseconds
 *  Helper macro to convert timeval struct to nanosecond values stored in a
 *  long long signed value (equivalent to int64_t)
 */
#define TIMEVAL2NSECS(x) static_cast<uint64_t>((x).tv_sec * 1000000000LL + (x).tv_usec * 1000LL)

/**
 *  \macro TIMEVAL2USECS
 *  Convert timeval struct to value in microseconds
 *  Helper macro to convert timeval struct to microsecond values stored in a
 *  long long signed value (equivalent to int64_t)
 */
#define TIMEVAL2USECS(x) (static_cast<uint64_t>( \
                            ((x).tv_sec * 1000000000LL + (x).tv_usec * 1000LL) / 1000LL))

// macro for float comparaion with 0
#define EPSILON 0.00001

// macro for the MAX FILENAME
#define MAX_SYS_NAME 64
#define MAX_TARGET_NAME 256

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private:declarations for a class
#ifndef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName&) = delete;    \
    TypeName& operator=(const TypeName&) = delete
#endif

/**
 *  Maker note maximum sizes
 *  Section 1 is used for Normal capture
 *  Section 2 is used for RAW captures
 */
#define MAKERNOTE_SECTION1_SIZE 56000
#define MAKERNOTE_SECTION2_SIZE 110592

// macro for memcpy
#ifndef MEMCPY_S
#define MEMCPY_S(dest, dmax, src, smax) \
    (void)memcpy((dest), (src), std::min(static_cast<size_t>(dmax), static_cast<size_t>(smax)))
#endif

#define STDCOPY(dst, src, size) std::copy((src), ((src) + (size)), (dst))

// macro for isys compression width/height calculation
#define ISYS_COMPRESSION_STRIDE_ALIGNMENT_BYTES 512
#define ISYS_COMPRESSION_HEIGHT_ALIGNMENT 1
#define ISYS_COMPRESSION_PAGE_SIZE 0x1000
// macro for psys compression width/height calculation
#define PSYS_COMPRESSION_PSA_Y_STRIDE_ALIGNMENT 256
#define PSYS_COMPRESSION_PSA_UV_STRIDE_ALIGNMENT 256
#define PSYS_COMPRESSION_PSA_HEIGHT_ALIGNMENT 2
#define PSYS_COMPRESSION_OFS_STRIDE_ALIGNMENT 128
#define PSYS_COMPRESSION_OFS_TILE_HEIGHT_ALIGNMENT 32
#define PSYS_COMPRESSION_OFS_LINEAR_HEIGHT_ALIGNMENT 2
#define PSYS_COMPRESSION_TNR_STRIDE_ALIGNMENT 128
#define PSYS_COMPRESSION_TNR_LINEAR_HEIGHT_ALIGNMENT 4

#define PSYS_COMPRESSION_PAGE_SIZE 0x1000
#define UV_STRIDE_DIVIDER 2
#define UV_HEIGHT_DIVIDER 2
// tile size definition
#define ISYS_COMPRESSION_TILE_SIZE_BYTES 512
#define TILE_SIZE_YUV420_Y 256
#define TILE_SIZE_YUV420_UV 128
#define TILE_SIZE_OFS10_12_TILEY 256
#define TILE_SIZE_OFS8_TILEY 256
#define TILE_SIZE_OFS8_10_LINEAR 128
#define TILE_SIZE_TNR_NV12_Y 512
#define TILE_SIZE_TNR_NV12_LINEAR 256

// tile status bits definition
#define ISYS_COMPRESSION_TILE_STATUS_BITS 4
#define TILE_STATUS_BITS_YUV420_Y 2
#define TILE_STATUS_BITS_YUV420_UV 1
#define TILE_STATUS_BITS_OFS_NV12_TILE_Y 8
#define TILE_STATUS_BITS_OFS_P010_TILE_Y 8
#define TILE_STATUS_BITS_OFS8_10_LINEAR 1
#define TILE_STATUS_BITS_TNR_NV12_TILE_Y 4
#define TILE_STATUS_BITS_TNR_NV12_LINEAR 2

#define CAMHAL_CEIL_DIV(a, b) (((a) + (b)-1) / (b))

#define FOURCC_TO_UL(a, b, c, d) \
    ((static_cast<uint32_t>(a)) | \
    ((static_cast<uint32_t>(b) << 8)) | \
    ((static_cast<uint32_t>(c) << 16)) | \
    ((static_cast<uint32_t>(d) << 24)))

// Internal useful tool for format
namespace CameraUtils {

int getFileContent(const char* filename, char* buffer, int maxSize);

const char* tuningMode2String(TuningMode mode);

TuningMode string2TuningMode(const char* str);

const char* pixelCode2String(int code);

int string2PixelCode(const char* code);

int string2IaFourccCode(const char* code);

std::string format2string(int format);

int string2format(const char* str);

const std::string fourcc2String(int format4cc);

unsigned int fourcc2UL(const char* str4cc);

bool isPlanarFormat(int format);

bool isRaw(int format);

int getBpp(int format);

int getStride(int format, int width);

int getPlanarByte(int format);

int32_t getBpl(int32_t fourcc, int32_t width);

int32_t getV4L2Format(const int32_t iaFourcc);
int32_t getFourccFormat(int32_t v4l2Fmt);

int getCompressedFrameSize(int format, int width, int height);

int getFrameSize(int format, int width, int height, bool needAlignedHeight = false,
                 bool needExtraSize = true, bool needCompression = false);

int getNumOfPlanes(int format);

void getDeviceName(const char* entityName, std::string& deviceNodeName, bool isSubDev = false);

void getSubDeviceName(const char* entityName, std::string& deviceNodeName);

int getInterlaceHeight(int field, int height);

bool isUllPsysPipe(TuningMode tuningMode);

ConfigMode getConfigModeByName(const char* ConfigName);

void getConfigModeFromString(std::string str, std::vector<ConfigMode>& cfgModes);

camera_scene_mode_t getSceneModeByName(const char* sceneName);

camera_awb_mode_t getAwbModeByName(const char* awbName);

unsigned int getMBusFormat(int cameraId, unsigned int isysFmt);

/**
 * Spit the given srcStr by delim into a vector of sub strings.
 */
std::vector<std::string> splitString(const char* srcStr, char delim);

nsecs_t systemTime();

void checkFps(unsigned int frameNumber, timeval *requestTime);

frame_usage_mode_t getFrameUsage(const stream_config_t *streamList);
}  // namespace CameraUtils

}  // namespace icamera

#endif // UTILS_H
