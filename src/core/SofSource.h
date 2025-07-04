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

#ifndef SOF_SOURCE_H
#define SOF_SOURCE_H

#include <vector>

#include <v4l2_device.h>
#include "CameraEvent.h"
#include "iutils/Thread.h"

namespace icamera {

// Event source for SOF event polled from subdevice.
class SofSource : public EventSource {
 public:
    SofSource(int cameraId);
    ~SofSource();
    int init();
    int deinit();
    int configure();
    int start();
    int stop();

    int poll();

 private:
    int initDev();
    int deinitDev();

    PollThread<SofSource>* mPollThread;
    int mCameraId;
    V4L2Subdevice* mIsysReceiverSubDev;
    bool mExitPending;
    bool mSofDisabled;
};

}  // namespace icamera

#endif // SOF_SOURCE_H
