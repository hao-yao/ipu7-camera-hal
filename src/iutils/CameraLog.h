/*
 * Copyright (C) 2015-2025 Intel Corporation.
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

#ifndef CAMERA_LOG_H
#define CAMERA_LOG_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "utils/ScopedAtrace.h"

#ifdef CAMERA_TRACE
#include "CameraTrace.h"
#endif

// ***********************************************************
struct GroupDesc {
    uint32_t level;
};

extern GroupDesc globalGroupsDescp[];

#include "LogSink.h"
#include "ModuleTags.h"

#define GENERATED_TAG_NAME(X) GENERATED_TAGS_##X
#define GET_FILE_SHIFT(X) GENERATED_TAG_NAME(X)

extern const char* tagNames[];
extern icamera::LogOutputSink* globalLogSink;

namespace icamera {

/* global log level */
extern uint32_t gLogLevel;
extern int gSlowlyRunRatio;

/**
 * LOG levels
 *
 * LEVEL 1 is used to track events in the HAL that are relevant during
 * the operation of the camera, but are not happening on a per frame basis.
 * this ensures that the level of logging is not too verbose
 *
 * LEVEL 2 is used to track information on a per request basis
 *
 * PERF TRACES enable only traces that provide performance metrics on the opera
 * tion of the HAL
 *
 * PERF TRACES BREAKDOWN provides further level of detail on the performance
 * metrics
 */
enum {
    /* verbosity level of general traces */
    // [0 - 3] bits
    CAMERA_DEBUG_LOG_LEVEL1 = 1U,
    CAMERA_DEBUG_LOG_LEVEL2 = 1U << 1,
    CAMERA_DEBUG_LOG_LEVEL3 = 1U << 2,
    // [4 - 7] bits
    CAMERA_DEBUG_LOG_INFO = 1U << 4,
    CAMERA_DEBUG_LOG_WARNING = 1U << 5,
    CAMERA_DEBUG_LOG_ERR = 1U << 6,
    // [8 - 11] bits
    CAMERA_DEBUG_LOG_CCA = 1U << 8,
    CAMERA_DEBUG_LOG_METADATA = 1U << 9,
    CAMERA_DEBUG_LOG_KERNEL_TOGGLE = 1U << 10,
};

enum {
    /* Emit well-formed performance traces */
    CAMERA_DEBUG_LOG_PERF_TRACES = 1U,

    /* Print out detailed timing analysis */
    CAMERA_DEBUG_LOG_PERF_TRACES_BREAKDOWN = 2U,

    /* Print out detailed timing analysis for IOCTL */
    CAMERA_DEBUG_LOG_PERF_IOCTL_BREAKDOWN = 1U << 2,

    /* Print out detailed memory information analysis for IOCTL */
    CAMERA_DEBUG_LOG_PERF_MEMORY = 1U << 3,

    /*enable camera atrace level 0 for camtune-record*/
    CAMERA_DEBUG_LOG_ATRACE_LEVEL0 = 1U << 4,

    // DUMP_ENTITY_TOPOLOGY_S
    /*enable media topology dump*/
    CAMERA_DEBUG_LOG_MEDIA_TOPO_LEVEL = 1U << 5,
    // DUMP_ENTITY_TOPOLOGY_E

    /*enable media controller info dump*/
    CAMERA_DEBUG_LOG_MEDIA_CONTROLLER_LEVEL = 1U << 6,

    /*enable camera imaging atrace level 1 for camtune-record*/
    CAMERA_DEBUG_LOG_ATRACE_LEVEL1 = 1U << 7,
};

enum {
    CAMERA_POWERBREAKDOWN_DISABLE_PREVIEW = 1U << 0,
    CAMERA_POWERBREAKDOWN_DISABLE_FDFR = 1U << 1,
    CAMERA_POWERBREAKDOWN_DISABLE_3A = 1U << 2,
};

const char* cameraDebugLogToString(uint32_t level);

namespace Log {
void setDebugLevel(void);
bool isDebugLevelEnable(uint32_t level);
bool isLogTagEnabled(int tag);
// DUMP_ENTITY_TOPOLOGY_S
bool isDumpMediaTopo(void);
// DUMP_ENTITY_TOPOLOGY_E
bool isDumpMediaInfo(void);
void ccaPrintError(const char* fmt, va_list ap);
void ccaPrintInfo(const char* fmt, va_list ap);
}  // namespace Log

#define SLOWLY_MULTIPLIER (icamera::gSlowlyRunRatio ? icamera::gSlowlyRunRatio : 1)

extern void doLogBody(int logTag, uint32_t level, int grpPosition, const char* fmt, ...);
extern void doLogBody(int logTag, uint32_t level, const char* fmt, ...);

#define LOG1(...)                                                                                \
    do {                                                                                         \
        { doLogBody(GET_FILE_SHIFT(LOG_TAG), icamera::CAMERA_DEBUG_LOG_LEVEL1, ##__VA_ARGS__); } \
    } while (0)

#define LOG2(...)                                                                                \
    do {                                                                                         \
        { doLogBody(GET_FILE_SHIFT(LOG_TAG), icamera::CAMERA_DEBUG_LOG_LEVEL2, ##__VA_ARGS__); } \
    } while (0)

#define LOG3(...)                                                                                \
    do {                                                                                         \
        { doLogBody(GET_FILE_SHIFT(LOG_TAG), icamera::CAMERA_DEBUG_LOG_LEVEL3, ##__VA_ARGS__); } \
    } while (0)

#define LOGI(...)                                                                              \
    do {                                                                                       \
        { doLogBody(GET_FILE_SHIFT(LOG_TAG), icamera::CAMERA_DEBUG_LOG_INFO, ##__VA_ARGS__); } \
    } while (0)

#define LOGE(...)                                                                             \
    do {                                                                                      \
        { doLogBody(GET_FILE_SHIFT(LOG_TAG), icamera::CAMERA_DEBUG_LOG_ERR, ##__VA_ARGS__); } \
    } while (0)

#define LOGW(...)                                                                                 \
    do {                                                                                          \
        { doLogBody(GET_FILE_SHIFT(LOG_TAG), icamera::CAMERA_DEBUG_LOG_WARNING, ##__VA_ARGS__); } \
    } while (0)

#define HAL_TRACE_NAME(level, name) ScopedTrace ___tracer(level, name)
#define HAL_TRACE_CALL(level) HAL_TRACE_NAME(level, __PRETTY_FUNCTION__)

#ifdef CAMERA_TRACE
#define CONCAT_(a, b) a##b
#define CONCAT(a, b) CONCAT_(a, b)
#define MAKE_COLOR(data) (icamera::CameraTrace::setColor(data))
#define TRACE_LOG_TYPE(type, ...) icamera::CameraTrace CONCAT(_Trace_, __LINE__)(type, __VA_ARGS__)

#define TRACE_LOG_PROCESS(...) TRACE_LOG_TYPE(icamera::TraceEventStart, __VA_ARGS__)
#define TRACE_STRUCT_PROCESS(name, struct_name, ...) \
    TRACE_LOG_TYPE(icamera::TraceEventStart, name, #struct_name, sizeof(struct_name), __VA_ARGS__)
#define TRACE_LOG_POINT(...) TRACE_LOG_TYPE(icamera::TraceEventPoint, __VA_ARGS__)
#define TRACE_STRUCT_POINT(name, struct_name, ...) \
    TRACE_LOG_TYPE(icamera::TraceEventPoint, name, #struct_name, sizeof(struct_name), __VA_ARGS__)

#else
#define MAKE_COLOR(data) (data)
#define TRACE_LOG_PROCESS(...)
#define TRACE_STRUCT_PROCESS(name, struct_name, ...)
#define TRACE_LOG_POINT(...)
#define TRACE_STRUCT_POINT(name, struct_name, ...)
#endif

class ScopedTrace {
 public:
    inline ScopedTrace(int level, const char* name) : mLevel(level), mName(name) {
        if (mLevel <= gLogLevel) {
            LOG1("ENTER-%s", name);
        }
    }

    inline ~ScopedTrace() {
        if (mLevel <= gLogLevel) {
            LOG1("EXIT-%s", mName);
        }
    }

 private:
    int mLevel;
    const char* mName;
};

}  // namespace icamera

#endif // CAMERA_LOG_H
