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

#ifndef CAMERA_EVENT_H
#define CAMERA_EVENT_H

#include <map>
#include <set>
#ifdef LIBCAMERA_BUILD
#include <libcamera/base/signal.h>
#endif
#include "CameraEventType.h"
#include "iutils/Thread.h"

namespace icamera {

class EventListener {
 public:
    EventListener() {}
    virtual ~EventListener() {}
    virtual void handleEvent(EventData eventData) {}
};

class EventSource {
 private:
#ifdef LIBCAMERA_BUILD
    libcamera::Signal<EventData> mNotifier[EVENT_TYPE_MAX];
#else
    std::map<EventType, std::set<EventListener*>> mListeners;

    // Guard for EventSource public API to protect mListeners.
    Mutex mListenersLock;
#endif

 public:
    EventSource() {}
    virtual ~EventSource() {}
    virtual void registerListener(EventType eventType, EventListener* eventListener);
    virtual void removeListener(EventType eventType, EventListener* eventListener);
    virtual void notifyListeners(EventData eventData);
};

}  // namespace icamera

#endif // CAMERA_EVENT_H
