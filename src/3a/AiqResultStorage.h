/*
 * Copyright (C) 2016-2025 Intel Corporation.
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

#ifndef AIQ_RESULT_STORAGE_H
#define AIQ_RESULT_STORAGE_H

#include <map>

#include "AiqResult.h"

#include "AiqStatistics.h"
#include "FaceType.h"
#include "iutils/Utils.h"
#include "iutils/Thread.h"
#include "iutils/RWLock.h"
#include "iutils/CameraDump.h"

namespace icamera {

/**
 * \class AiqResultStorage
 *
 * This class provides interfaces for setting and getting AiqResult, and a storage space
 * which is able to contain at most `kStorageSize` AiqResults at same time.
 *
 * It's a singleton based on camera id, and its life cycle can be maintained by
 * its static methods getInstance and releaseAiqResultStorage.
 */
class AiqResultStorage {
public:
    /**
     * \brief Acquire Aiq result.
     *
     * The function will return one Aiq result pointer which is kept by Aiq algo.
     * The sequence id is set to -1 which indicates the Aiq result is invalid.
     *
     * return Aiq result pointer to be kept by Aiq algo.
     */
    AiqResult* acquireAiqResult();

    /**
     * \brief Update mCurrentIndex and set sequence id into internal storage.
     */
    void updateAiqResult(int64_t sequence);

    /**
     * \brief Get the pointer of aiq result to internal storage by given sequence id.
     *
     * The function will return the internal pointer of AiqResult, the caller MUST use this
     * pointer quickly, let's say less than 10ms. For any time-consuming operations, it's
     * the caller's responsibility to do a deep-copy, otherwise the data in returned AiqResult
     * may not be consistent.
     *
     * param[in] int64_t sequence: specify which aiq result is needed.
     *
     * return 1. when sequence id is -1 or not provided, the lastest result will be returned.
     *        2. when sequence id is larger than -1, the result with gaven sequence id will be returned.
     *        3. if cannot find in result storage, it means either sequence id is too old and its
     *           result was overrided, or the sequence id is too new, and its result has not been
     *           saved into storage yet. For both cases, nullptr will be returned.
     */
    const AiqResult* getAiqResult(int64_t sequence = -1);

    /**
     * \brief Acquire AIQ statistics.
     *
     * The function will return one AIQ statistics pointer which is kept by AIQ statistics decoder.
     * The sequence id is set to -1 which indicates the AIQ statistics is invalid.
     *
     * return AIQ statistics pointer to be kept by AIQ statistics decoder..
     */
    AiqStatistics* acquireAiqStatistics();

    /**
     * \brief Update mCurrentAiqStatsIndex and set sequence id in internal storage.
     */
    void updateAiqStatistics(int64_t sequence);

    /**
     * \brief Get the pointer of AIQ statistics to internal storage.
     *
     * The function will return the latest AIQ statistics, and set the mInUse flag to true.
     *
     * return the latest AIQ statistics.
     */
    const AiqStatistics* getAndLockAiqStatistics();

    /**
     * \brief Clear the mInUse flag of all the AIQ statitics in internal storage.
     */
    void unLockAiqStatistics();

    /**
     * \brief Reset mCurrentAiqStatsIndex after restart AIQ.
     */
    void resetAiqStatistics();

    /**
     * \brief Acquire a FaceDetectionResult buffer
     */
    FaceDetectionResult* acquireFaceResult();

    /**
     * \brief update the index and set sequence for FaceDetectionResult
     */
    void updateFaceResult(int64_t sequence);

    /**
     * \brief get the latest FaceDetectionResult
     */
    FaceDetectionResult* getFaceResult();

    AiqResultStorage(int cameraId);
    ~AiqResultStorage();

private:
    int mCameraId;
    RWLock mDataLock;   // lock for all the data storage below

    static const int kStorageSize = MAX_SETTING_COUNT; // Should > MAX_BUFFER_COUNT + sensorLag
    int mCurrentIndex = -1;
    AiqResult* mAiqResults[kStorageSize];

    static const int kAiqStatsStorageSize = 3; // Always use the latest, but may hold for long time
    int mCurrentAiqStatsIndex = -1;
    AiqStatistics mAiqStatistics[kAiqStatsStorageSize];

    static const int kFaceResultStorageSize = 3;  // Always use the latest
    int mCurrentFaceResultIndex = -1;
    FaceDetectionResult mFaceResult[kFaceResultStorageSize];
    RWLock mFaceLock;
};

} //namespace icamera

#endif // AIQ_RESULT_STORAGE_H
