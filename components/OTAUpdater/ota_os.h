/**
 * @file ota_os.h
 * @brief OTA Updater component OS abstraction layer
 * 
 * Provides RTOS primitives for the OTA update task.
 * Uses the core os_primitives.h API.
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 */

#ifndef OTA_OS_H
#define OTA_OS_H

#include "os_primitives.h"

namespace OTAOS {

// OTA task configuration
constexpr size_t TASK_STACK_SIZE = 8192;  // 8KB stack (HTTP/TLS requires more stack)
constexpr os::Priority TASK_PRIORITY = os::Priority::MEDIUM;
constexpr const char* TASK_NAME = "ota_task";

/**
 * @brief Create OTA update task
 * @param taskFunc Task function to execute (OTA download and apply)
 * @param outHandle Optional output task handle
 * @return os::Result::OK on success
 */
inline os::Result createOTATask(std::function<void()> taskFunc, os::TaskHandle* outHandle = nullptr) {
    os::TaskConfig config(TASK_NAME, taskFunc, TASK_STACK_SIZE, TASK_PRIORITY);
    return os::taskCreate(config, outHandle);
}

/**
 * @brief Delete current task (OTA task self-deletes when complete)
 */
inline void deleteSelf() {
    os::taskDelete(os::TaskHandle()); // nullptr = current task
}

} // namespace OTAOS

#endif // OTA_OS_H
