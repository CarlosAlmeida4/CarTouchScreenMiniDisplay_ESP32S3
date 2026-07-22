/**
 * @file os_primitives.h
 * @brief RTOS abstraction layer for portable embedded applications
 * 
 * Provides a unified interface for common RTOS primitives (tasks, queues, mutexes,
 * semaphores, delays) that abstracts FreeRTOS, Zephyr, and other RTOS implementations.
 * 
 * Design Philosophy:
 * - Minimal API surface (only abstract what components use)
 * - Type-safe where possible (C++ templates)
 * - Explicit error handling (return codes)
 * - Zero-overhead abstractions (inline, compile-time dispatch)
 * - RAII patterns for resource management
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 * @license MIT License
 */

#ifndef OS_PRIMITIVES_H
#define OS_PRIMITIVES_H

#include <cstddef>
#include <cstdint>
#include <functional>

// Forward declare backend types (actual types defined in backend implementation)
namespace os {
namespace impl {
    struct TaskHandleImpl;
    struct QueueHandleImpl;
    struct MutexHandleImpl;
    struct SemaphoreHandleImpl;
}
}

namespace os {

// ============================================================================
// PRIORITY ABSTRACTION
// ============================================================================

/**
 * @brief Abstract task priority levels (shields from RTOS-specific numbering)
 * 
 * FreeRTOS: Higher number = higher priority (0 = lowest, configMAX_PRIORITIES-1 = highest)
 * Zephyr:   Lower number = higher priority (0 = highest, negatives = cooperative)
 * 
 * This abstraction maps to appropriate backend values automatically.
 */
enum class Priority : uint8_t {
    IDLE = 0,      ///< Lowest priority (background tasks)
    LOW = 1,       ///< Low priority (non-critical periodic tasks)
    MEDIUM = 2,    ///< Medium priority (UI, sensors)
    HIGH = 3,      ///< High priority (networking, time-critical)
    REALTIME = 4   ///< Highest priority (ISR-deferred, critical control)
};

// ============================================================================
// ERROR CODES
// ============================================================================

/**
 * @brief RTOS operation result codes
 */
enum class Result : int8_t {
    OK = 0,           ///< Operation succeeded
    TIMEOUT = -1,     ///< Operation timed out
    WOULD_BLOCK = -2, ///< Operation would block (non-blocking mode)
    NO_MEMORY = -3,   ///< Insufficient memory for allocation
    INVALID = -4,     ///< Invalid parameter
    ERROR = -5        ///< Generic error
};

// Helper macros for error checking
#define OS_CHECK(expr) do { \
    os::Result _res = (expr); \
    if (_res != os::Result::OK) { \
        return _res; \
    } \
} while(0)

#define OS_ASSERT(expr) do { \
    if (!(expr)) { \
        os::panic("OS assertion failed: " #expr); \
    } \
} while(0)

// ============================================================================
// TASK API
// ============================================================================

/**
 * @brief Opaque task handle (platform-agnostic)
 */
class TaskHandle {
public:
    TaskHandle() : impl_(nullptr) {}
    explicit TaskHandle(impl::TaskHandleImpl* impl) : impl_(impl) {}
    
    bool isValid() const { return impl_ != nullptr; }
    impl::TaskHandleImpl* getImpl() const { return impl_; }
    
private:
    impl::TaskHandleImpl* impl_;
};

/**
 * @brief Task configuration structure
 */
struct TaskConfig {
    const char* name;           ///< Task name (for debugging)
    std::function<void()> func; ///< Task entry point
    size_t stackSize;           ///< Stack size in bytes
    Priority priority;          ///< Task priority
    void* parameter;            ///< Optional user parameter (nullptr if unused)
    
    TaskConfig(const char* n, std::function<void()> f, size_t stack, Priority prio)
        : name(n), func(f), stackSize(stack), priority(prio), parameter(nullptr) {}
};

/**
 * @brief Create and start a new task
 * @param config Task configuration
 * @param outHandle Output task handle (optional, can be nullptr)
 * @return Result::OK on success, error code on failure
 */
Result taskCreate(const TaskConfig& config, TaskHandle* outHandle = nullptr);

/**
 * @brief Delete a task (can be called from task itself to self-delete)
 * @param handle Task handle (nullptr = current task)
 * @return Result::OK on success, error code on failure
 */
Result taskDelete(TaskHandle handle);

/**
 * @brief Delay current task for specified milliseconds
 * @param ms Delay duration in milliseconds
 */
void taskDelay(uint32_t ms);

/**
 * @brief Get tick count in milliseconds (monotonic, wraps at ~49 days)
 * @return Current tick count
 */
uint32_t getTickCount();

// ============================================================================
// QUEUE API
// ============================================================================

/**
 * @brief Opaque queue handle (platform-agnostic)
 */
class QueueHandle {
public:
    QueueHandle() : impl_(nullptr) {}
    explicit QueueHandle(impl::QueueHandleImpl* impl) : impl_(impl) {}
    
    bool isValid() const { return impl_ != nullptr; }
    impl::QueueHandleImpl* getImpl() const { return impl_; }
    
private:
    impl::QueueHandleImpl* impl_;
};

/**
 * @brief Create a queue with specified item size and capacity
 * @param itemSize Size of each queue item in bytes
 * @param capacity Maximum number of items queue can hold
 * @param outHandle Output queue handle
 * @return Result::OK on success, error code on failure
 */
Result queueCreate(size_t itemSize, size_t capacity, QueueHandle* outHandle);

/**
 * @brief Send item to back of queue (blocking with timeout)
 * @param handle Queue handle
 * @param item Pointer to item data to copy into queue
 * @param timeoutMs Timeout in milliseconds (0 = no wait, UINT32_MAX = wait forever)
 * @return Result::OK on success, Result::TIMEOUT if timed out
 */
Result queueSend(QueueHandle handle, const void* item, uint32_t timeoutMs = 0);

/**
 * @brief Receive item from front of queue (blocking with timeout)
 * @param handle Queue handle
 * @param item Pointer to buffer where received item will be copied
 * @param timeoutMs Timeout in milliseconds (0 = no wait, UINT32_MAX = wait forever)
 * @return Result::OK on success, Result::TIMEOUT if timed out
 */
Result queueReceive(QueueHandle handle, void* item, uint32_t timeoutMs = 0);

/**
 * @brief Overwrite queue with single item (always succeeds, replaces existing data)
 * @param handle Queue handle (must be queue of length 1)
 * @param item Pointer to item data to overwrite queue with
 * @return Result::OK on success, error code on failure
 */
Result queueOverwrite(QueueHandle handle, const void* item);

/**
 * @brief Get number of items currently in queue
 * @param handle Queue handle
 * @return Number of items waiting in queue
 */
size_t queueWaiting(QueueHandle handle);

/**
 * @brief Delete a queue and free resources
 * @param handle Queue handle
 */
void queueDelete(QueueHandle handle);

// ============================================================================
// TYPE-SAFE QUEUE WRAPPERS (C++ templates)
// ============================================================================

/**
 * @brief Type-safe queue wrapper (compile-time type checking)
 * @tparam T Item type (must be trivially copyable)
 */
template<typename T>
class Queue {
public:
    Queue() : handle_() {}
    
    /**
     * @brief Create queue with specified capacity
     * @param capacity Maximum number of items
     * @return Result::OK on success
     */
    Result create(size_t capacity) {
        return queueCreate(sizeof(T), capacity, &handle_);
    }
    
    /**
     * @brief Send item to queue
     * @param item Item to send (copied into queue)
     * @param timeoutMs Timeout in milliseconds
     * @return Result::OK on success
     */
    Result send(const T& item, uint32_t timeoutMs = 0) {
        return queueSend(handle_, &item, timeoutMs);
    }
    
    /**
     * @brief Receive item from queue
     * @param item Output item buffer
     * @param timeoutMs Timeout in milliseconds
     * @return Result::OK on success
     */
    Result receive(T& item, uint32_t timeoutMs = 0) {
        return queueReceive(handle_, &item, timeoutMs);
    }
    
    /**
     * @brief Overwrite queue (for single-item queues)
     * @param item Item to overwrite with
     * @return Result::OK on success
     */
    Result overwrite(const T& item) {
        return queueOverwrite(handle_, &item);
    }
    
    size_t waiting() const { return queueWaiting(handle_); }
    bool isValid() const { return handle_.isValid(); }
    QueueHandle getHandle() const { return handle_; }
    
    ~Queue() {
        if (handle_.isValid()) {
            queueDelete(handle_);
        }
    }
    
private:
    QueueHandle handle_;
};

// ============================================================================
// MUTEX API
// ============================================================================

/**
 * @brief Opaque mutex handle (platform-agnostic)
 */
class MutexHandle {
public:
    MutexHandle() : impl_(nullptr) {}
    explicit MutexHandle(impl::MutexHandleImpl* impl) : impl_(impl) {}
    
    bool isValid() const { return impl_ != nullptr; }
    impl::MutexHandleImpl* getImpl() const { return impl_; }
    
private:
    impl::MutexHandleImpl* impl_;
};

/**
 * @brief Create a mutex
 * @param outHandle Output mutex handle
 * @return Result::OK on success, error code on failure
 */
Result mutexCreate(MutexHandle* outHandle);

/**
 * @brief Lock mutex (blocking)
 * @param handle Mutex handle
 * @param timeoutMs Timeout in milliseconds (UINT32_MAX = wait forever)
 * @return Result::OK on success, Result::TIMEOUT if timed out
 */
Result mutexLock(MutexHandle handle, uint32_t timeoutMs = UINT32_MAX);

/**
 * @brief Unlock mutex
 * @param handle Mutex handle
 * @return Result::OK on success
 */
Result mutexUnlock(MutexHandle handle);

/**
 * @brief Delete mutex and free resources
 * @param handle Mutex handle
 */
void mutexDelete(MutexHandle handle);

/**
 * @brief RAII mutex lock guard (automatic unlock on scope exit)
 */
class MutexGuard {
public:
    explicit MutexGuard(MutexHandle handle) : handle_(handle) {
        mutexLock(handle_);
    }
    
    ~MutexGuard() {
        mutexUnlock(handle_);
    }
    
    // Non-copyable, non-movable
    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;
    
private:
    MutexHandle handle_;
};

// ============================================================================
// SEMAPHORE API
// ============================================================================

/**
 * @brief Opaque semaphore handle (platform-agnostic)
 */
class SemaphoreHandle {
public:
    SemaphoreHandle() : impl_(nullptr) {}
    explicit SemaphoreHandle(impl::SemaphoreHandleImpl* impl) : impl_(impl) {}
    
    bool isValid() const { return impl_ != nullptr; }
    impl::SemaphoreHandleImpl* getImpl() const { return impl_; }
    
private:
    impl::SemaphoreHandleImpl* impl_;
};

/**
 * @brief Create a binary semaphore (0 or 1 count)
 * @param outHandle Output semaphore handle
 * @return Result::OK on success, error code on failure
 */
Result semaphoreCreateBinary(SemaphoreHandle* outHandle);

/**
 * @brief Wait on semaphore (blocking with timeout)
 * @param handle Semaphore handle
 * @param timeoutMs Timeout in milliseconds (0 = no wait, UINT32_MAX = wait forever)
 * @return Result::OK on success, Result::TIMEOUT if timed out
 */
Result semaphoreTake(SemaphoreHandle handle, uint32_t timeoutMs = UINT32_MAX);

/**
 * @brief Signal semaphore (increment count)
 * @param handle Semaphore handle
 * @return Result::OK on success
 */
Result semaphoreGive(SemaphoreHandle handle);

/**
 * @brief Delete semaphore and free resources
 * @param handle Semaphore handle
 */
void semaphoreDelete(SemaphoreHandle handle);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Platform-specific panic handler (called on assertion failure)
 * @param message Error message
 */
[[noreturn]] void panic(const char* message);

/**
 * @brief Initialize RTOS abstraction layer (called once at startup)
 * Must be called before any other os:: functions.
 */
void init();

} // namespace os

#endif // OS_PRIMITIVES_H
