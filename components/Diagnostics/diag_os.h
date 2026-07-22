/**
 * @file diag_os.h
 * @brief Diagnostics component OS abstraction layer
 * 
 * Provides RTOS primitives for diagnostics HTTP server (delays for cooperative multitasking).
 * Uses the core os_primitives.h API.
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 */

#ifndef DIAG_OS_H
#define DIAG_OS_H

#include "os_primitives.h"

namespace DiagOS {

// Diagnostics timing
constexpr uint32_t HTTP_YIELD_DELAY_MS = 1;       // 1ms yield to allow LVGL task to run
constexpr uint32_t SERVER_START_DELAY_MS = 500;   // 500ms delay before starting server (port release)
constexpr uint32_t RETRY_DELAY_MS = 200;          // 200ms retry delay for server start

/**
 * @brief Yield to LVGL task (prevents watchdog timeout in HTTP handlers)
 * Called periodically during long HTTP response generation
 */
inline void yieldToLVGL() {
    os::taskDelay(HTTP_YIELD_DELAY_MS);
}

/**
 * @brief Server start delay (allows TIME_WAIT to expire)
 */
inline void serverStartDelay() {
    os::taskDelay(SERVER_START_DELAY_MS);
}

/**
 * @brief Retry delay for server start attempts
 */
inline void retryDelay() {
    os::taskDelay(RETRY_DELAY_MS);
}

} // namespace DiagOS

#endif // DIAG_OS_H
