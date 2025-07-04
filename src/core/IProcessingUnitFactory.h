/*
 * Copyright (C) 2022-2025 Intel Corporation.
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

#ifndef IPROCESSING_UNIT_FACTORY_H
#define IPROCESSING_UNIT_FACTORY_H

#include "AiqUnit.h"
#include "CameraScheduler.h"
#include "IProcessingUnit.h"

namespace icamera {

/*
 * \factory class I3AControlFactory
 * This class is used to create the right instance of ProcessingUnit
 */
class IProcessingUnitFactory {
 public:
    /**
     * \brief Select the ProcessingUnit according to streamList and compiling option
     *
     * \param cameraId: the camera id
     * \param streamList: the stream list requested by user
     *
     * \return the IProcessingUnit class
     */
    static IProcessingUnit* createIProcessingUnit(int cameraId,
                                                  std::shared_ptr<CameraScheduler> scheduler);
};
}  // namespace icamera

#endif // IPROCESSING_UNIT_FACTORY_H
