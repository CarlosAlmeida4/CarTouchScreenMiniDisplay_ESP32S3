/**
 * @file qmi_os.h
 * @brief QMI8658c IMU component OS abstraction layer
 * 
 * Provides RTOS primitives for the QMI8658c sensor interface (task, queue, delays).
 * Uses the core os_primitives.h API.
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 */

#ifndef QMI_OS_H
#define QMI_OS_H

#include "os_primitives.h"
#include "PipelineTypes.hpp"

namespace QMIOS {

// QMI sensor task configuration
constexpr size_t TASK_STACK_SIZE = 4096;  // 4KB stack
constexpr os::Priority TASK_PRIORITY = os::Priority::HIGH;  // High priority for sensor timing
constexpr const char* TASK_NAME = "qmi_sensor_task";

// Sensor timing
constexpr uint32_t QMI_TASK_PERIOD_MS = 40;  // 40ms sensor read period (25Hz)

/**
 * @brief Create QMI sensor task
 * @param taskFunc Task function to execute
 * @param outHandle Optional output task handle
 * @return os::Result::OK on success
 */
inline os::Result createSensorTask(std::function<void()> taskFunc, os::TaskHandle* outHandle = nullptr) {
    os::TaskConfig config(TASK_NAME, taskFunc, TASK_STACK_SIZE, TASK_PRIORITY);
    return os::taskCreate(config, outHandle);
}

/**
 * @brief Delete current task (called on sensor init failure)
 */
inline void deleteSelf() {
    os::taskDelete(os::TaskHandle()); // nullptr = current task
}

/**
 * @brief Send sensor data to queue (overwrite mode - always succeeds)
 * @param queue Queue handle
 * @param data Sensor data to send
 * @return os::Result::OK on success
 */
inline os::Result sendSensorData(os::QueueHandle queue, const RollPitch& data) {
    return os::queueOverwrite(queue, &data);
}

/**
 * @brief Periodic delay for sensor reading loop
 */
inline void sensorDelay() {
    os::taskDelay(QMI_TASK_PERIOD_MS);
}

} // namespace QMIOS

#endif // QMI_OS_H
