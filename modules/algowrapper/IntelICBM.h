/*
 * Copyright (C) 2022-2023 Intel Corporation
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

#include <vector>
#include <memory>

#include "src/icbm/OnePunchIC2.h"
#include "src/icbm/ICBMTypes.h"

namespace icamera {

class IntelICBM {
 public:
    int setup(ICBMInitInfo* initParam, std::shared_ptr<IC2ApiHandle> handle = nullptr);
    int shutdown(const ICBMReqInfo& reqInfo);

    int processFrame(const ICBMReqInfo& reqInfo);
    void* allocBuffer(uint32_t bufSize, int id);
    void freeAllBufs();
 private:
    IntelOPIC2* mIntelOPIC2;
    std::vector<void*> mClientBufMems;
};
}  // namespace icamera
