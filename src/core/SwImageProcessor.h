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

#ifndef SW_IMAGE_PROCESSOR_H
#define SW_IMAGE_PROCESSOR_H

#include "IProcessingUnit.h"

namespace icamera {

/**
  * SwImageProcessor runs the Image Process Alogirhtm in the CPU.
  * It implements the BufferConsumer and BufferProducer Interface
  * This class is for debug purpose when the PsysProcess is not ready.
  */
class SwImageProcessor: public IProcessingUnit {
public:
    SwImageProcessor(int cameraId);
    virtual ~SwImageProcessor();

    /**
     * \brief Buffer producer Interface
     */
    virtual int     start();
    virtual void    stop();

private:
    int processNewFrame();

private:
    int mCameraId;
};

} //namespace icamera

#endif // SW_IMAGE_PROCESSOR_H
