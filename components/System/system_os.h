/**
 * @file system_os.h
 * @brief System component OS abstraction layer
 * 
 * Provides RTOS primitives specific to the System component.
 * Uses the core os_primitives.h API.
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 */

#ifndef SYSTEM_OS_H
#define SYSTEM_OS_H

#include "os_primitives.h"
#include "PipelineTypes.hpp"

namespace SystemOS {

/**
 * @brief Create queues for inter-component communication
 * @param rollPitchQueue Output queue for sensor data
 * @param wifiQueue Output queue for WiFi status
 * @return os::Result::OK on success
 */
inline os::Result createQueues(os::Queue<RollPitch>& rollPitchQueue, 
                                os::Queue<WifiManagerPipeline>& wifiQueue) {
    // Create queue for roll/pitch sensor data (capacity=1, latest value only)
    auto result = rollPitchQueue.create(1);
    if (result != os::Result::OK) {
        return result;
    }
    
    // Create queue for WiFi manager status (capacity=1, latest status only)
    result = wifiQueue.create(1);
    if (result != os::Result::OK) {
        return result;
    }
    
    return os::Result::OK;
}

} // namespace SystemOS

#endif // SYSTEM_OS_H
