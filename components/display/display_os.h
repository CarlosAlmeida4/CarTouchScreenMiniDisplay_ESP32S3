/**
 * @file display_os.h
 * @brief Display component OS abstraction layer
 * 
 * Provides RTOS primitives for the Display component (LVGL task, queues, semaphores).
 * Uses the core os_primitives.h API.
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 */

#ifndef DISPLAY_OS_H
#define DISPLAY_OS_H

#include "os_primitives.h"
#include "PipelineTypes.hpp"

namespace DisplayOS {

// Display task configuration constants
constexpr size_t TASK_STACK_SIZE = 1024 * 4;  // 4KB stack
constexpr os::Priority TASK_PRIORITY = os::Priority::MEDIUM;
constexpr const char* TASK_NAME = "display_task";

// LVGL timing constants
constexpr uint32_t LVGL_TICK_PERIOD_MS = 5;
constexpr uint32_t LVGL_TASK_MAX_DELAY_MS = 500;
constexpr uint32_t LVGL_TASK_MIN_DELAY_MS = 1;

/**
 * @brief LVGL mutex for thread-safe access to LVGL functions
 */
class LVGLMutex {
public:
    LVGLMutex() {
        os::semaphoreCreateBinary(&semaphore_);
        os::semaphoreGive(semaphore_); // Initially unlocked
    }
    
    ~LVGLMutex() {
        os::semaphoreDelete(semaphore_);
    }
    
    /**
     * @brief Lock LVGL mutex with timeout
     * @param timeoutMs Timeout in milliseconds (-1 = wait forever)
     * @return true if locked, false if timeout
     */
    bool lock(int timeoutMs = -1) {
        uint32_t timeout = (timeoutMs < 0) ? UINT32_MAX : static_cast<uint32_t>(timeoutMs);
        return os::semaphoreTake(semaphore_, timeout) == os::Result::OK;
    }
    
    /**
     * @brief Unlock LVGL mutex
     */
    void unlock() {
        os::semaphoreGive(semaphore_);
    }
    
    os::SemaphoreHandle getHandle() const { return semaphore_; }
    
private:
    os::SemaphoreHandle semaphore_;
};

/**
 * @brief Create display task
 * @param taskFunc Task function to execute
 * @param outHandle Optional output task handle
 * @return os::Result::OK on success
 */
inline os::Result createDisplayTask(std::function<void()> taskFunc, os::TaskHandle* outHandle = nullptr) {
    os::TaskConfig config(TASK_NAME, taskFunc, TASK_STACK_SIZE, TASK_PRIORITY);
    return os::taskCreate(config, outHandle);
}

/**
 * @brief Receive sensor data from queue (non-blocking)
 * @param queue Queue handle
 * @param data Output data buffer
 * @return os::Result::OK if data received, os::Result::TIMEOUT if empty
 */
inline os::Result receiveSensorData(os::QueueHandle queue, RollPitch& data) {
    return os::queueReceive(queue, &data, 0); // Non-blocking
}

/**
 * @brief Receive WiFi status from queue (non-blocking)
 * @param queue Queue handle
 * @param status Output status buffer
 * @return os::Result::OK if data received, os::Result::TIMEOUT if empty
 */
inline os::Result receiveWifiStatus(os::QueueHandle queue, WifiManagerPipeline& status) {
    return os::queueReceive(queue, &status, 0); // Non-blocking
}

/**
 * @brief LVGL tick delay (5ms periodic)
 */
inline void lvglTickDelay() {
    os::taskDelay(LVGL_TICK_PERIOD_MS);
}

} // namespace DisplayOS

#endif // DISPLAY_OS_H
