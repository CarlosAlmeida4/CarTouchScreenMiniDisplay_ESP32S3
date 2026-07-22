/**
 * @file wifi_os.h
 * @brief WiFi Manager component OS abstraction layer
 * 
 * Provides RTOS primitives for WiFi manager and provisioning tasks.
 * Uses the core os_primitives.h API.
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 */

#ifndef WIFI_OS_H
#define WIFI_OS_H

#include "os_primitives.h"
#include "PipelineTypes.hpp"

namespace WifiOS {

// WiFi manager task configuration
constexpr size_t MANAGER_TASK_STACK_SIZE = 4096;  // 4KB stack
constexpr os::Priority MANAGER_TASK_PRIORITY = os::Priority::MEDIUM;
constexpr const char* MANAGER_TASK_NAME = "wifi_manager_task";

// WiFi provisioning task configuration
constexpr size_t PROV_TASK_STACK_SIZE = 4096;  // 4KB stack
constexpr os::Priority PROV_TASK_PRIORITY = os::Priority::MEDIUM;
constexpr const char* PROV_TASK_NAME = "wifi_prov_task";

// WiFi manager timing
constexpr uint32_t MANAGER_LOOP_DELAY_MS = 1000;  // 1s state machine loop
constexpr uint32_t RETRY_DELAY_MS = 200;          // 200ms retry delay
constexpr uint32_t SERVER_START_DELAY_MS = 500;   // 500ms delay before starting server

/**
 * @brief Create WiFi manager task
 * @param taskFunc Task function to execute
 * @param outHandle Optional output task handle
 * @return os::Result::OK on success
 */
inline os::Result createManagerTask(std::function<void()> taskFunc, os::TaskHandle* outHandle = nullptr) {
    os::TaskConfig config(MANAGER_TASK_NAME, taskFunc, MANAGER_TASK_STACK_SIZE, MANAGER_TASK_PRIORITY);
    return os::taskCreate(config, outHandle);
}

/**
 * @brief Create WiFi provisioning task
 * @param taskFunc Task function to execute
 * @param outHandle Optional output task handle
 * @return os::Result::OK on success
 */
inline os::Result createProvisioningTask(std::function<void()> taskFunc, os::TaskHandle* outHandle = nullptr) {
    os::TaskConfig config(PROV_TASK_NAME, taskFunc, PROV_TASK_STACK_SIZE, PROV_TASK_PRIORITY);
    return os::taskCreate(config, outHandle);
}

/**
 * @brief Send WiFi status to queue (overwrite mode)
 * @param queue Queue handle
 * @param status WiFi status to send
 * @return os::Result::OK on success
 */
inline os::Result sendWifiStatus(os::QueueHandle queue, const WifiManagerPipeline& status) {
    return os::queueOverwrite(queue, &status);
}

/**
 * @brief Manager loop delay (1000ms)
 */
inline void managerLoopDelay() {
    os::taskDelay(MANAGER_LOOP_DELAY_MS);
}

/**
 * @brief Retry delay (200ms)
 */
inline void retryDelay() {
    os::taskDelay(RETRY_DELAY_MS);
}

/**
 * @brief Server start delay (500ms - allows port to release)
 */
inline void serverStartDelay() {
    os::taskDelay(SERVER_START_DELAY_MS);
}

} // namespace WifiOS

#endif // WIFI_OS_H
