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

#pragma once

#include <map>
#include <string>
#include <vector>

#ifndef IPU_SYSVER_ipu7
extern "C" {
#include <ia_cipf_css/ia_cipf_css.h>
}
#endif

namespace icamera {

#ifdef IPU_SYSVER_ipu7  // For ipu7
#define STAGE_UID(stream, stage)          ((((stream)&0xFFFF) << 16) | (((stage)&0xFF) << 8))
#define PORT_UID(stream, stage, terminal) (STAGE_UID(stream, stage) + ((terminal) & 0xFF)+ 1)

#define GET_STREAM_ID(uuid)   (((uuid) >> 16) & 0xFFFF)
#define GET_STAGE_ID(uuid)    (((uuid) >> 8) & 0xFF)
#define GET_TERMINAL_ID(uuid) (((uuid) & 0xFF) - 1)

#else  // for ipu6
// UID of ipu stage has no stream id info in ipu6 platform
#define STAGE_UID(stream, stage)          psys_2600_pg_uid(stage)
#define PORT_UID(stream, stage, terminal) (STAGE_UID(stream, stage) + (terminal) + 1)
// Don't support gpu stage now, because it uses different difinition in ia_cipf_common.h
// #define ia_cipf_external_stage_uid(id) ia_fourcc(((id & 0xFF00) >> 8),id,'E','X')

#define GET_STAGE_ID(uuid)    (((uuid) >> 16) & 0xFFFF)
#define GET_TERMINAL_ID(uuid) (uuid - psys_2600_pg_uid(GET_STAGE_ID(uuid)) - 1)
#endif

#ifdef IPU_SYSVER_ipu7
// IPU stage start with 0x10
#define IPU_STAGE_ID_BASE   0x10
#endif

// GPU stage start with 0x80
#define GPU_TNR_STAGE_ID    0x80
#define GPU_EVCP_STAGE_ID   0x81

// CPU SW post stage start with 0x90
#define SW_POST_STAGE_ID_BASE     0x90  // 0x90 for still pipe, 0x91~0x92 for video pipe

#define SW_POST_STAGE_NAME_BASE   "post_"
#define GPU_POST_STAGE_NAME_BASE  "gpu_"

#define POST_STAGE_INPUT       0
#define POST_STAGE_OUTPUT_BASE 1
#define POST_STAGE_OUTPUT_1    POST_STAGE_OUTPUT_BASE
#define POST_STAGE_OUTPUT_2    POST_STAGE_OUTPUT_BASE + 1
#define POST_STAGE_OUTPUT_3    POST_STAGE_OUTPUT_BASE + 2

#define SW_POST_REPROCESSING_STAGE_ID   0x98
#define SW_POST_REPROCESSING_STAGE_NAME "post_yuv"

#define YUV_REPROCESSING_STREAM_ID      70000

// Define stream id for ISYS and user port because they have no stream id in graph
#define IPU_ISYS_STREAM_ID  0
#define ISYS_STAGE_ID       0x1
#define ISYS_STAGE_UID                  STAGE_UID(IPU_ISYS_STREAM_ID, ISYS_STAGE_ID)
#define INPUT_STREAM_PORT_UID(terminal) PORT_UID(IPU_ISYS_STREAM_ID, ISYS_STAGE_ID, terminal)
#define MAIN_INPUT_PORT_UID             INPUT_STREAM_PORT_UID(0)
#define YUV_REPROCESSING_INPUT_STAGE_ID 0x3
#define YUV_REPROCESSING_INPUT_PORT_ID \
        PORT_UID(IPU_ISYS_STREAM_ID, YUV_REPROCESSING_INPUT_STAGE_ID, 0)

#define USER_PORT_STREAM_ID 0
#define USER_STAGE_ID       0x2
#define USER_STREAM_STAGE_UID           STAGE_UID(USER_PORT_STREAM_ID, USER_STAGE_ID)
#define USER_STREAM_PORT_UID(terminal)  PORT_UID(USER_PORT_STREAM_ID, USER_STAGE_ID, terminal)
#define USER_DEFAULT_PORT_UID           USER_STREAM_PORT_UID(0)

}  // namespace icamera
