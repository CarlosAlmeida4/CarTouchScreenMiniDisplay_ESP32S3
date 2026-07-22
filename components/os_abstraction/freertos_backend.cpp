/**
 * @file freertos_backend.cpp
 * @brief FreeRTOS backend implementation for os_primitives.h
 * 
 * Implements the RTOS abstraction layer using ESP-IDF FreeRTOS APIs.
 * 
 * @copyright 2026 CarTouchScreenMiniDisplay Project
 * @license MIT License
 */

#include "os_primitives.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include <cstring>

static const char* TAG = "OS_Abstraction";

namespace os {
namespace impl {

// ============================================================================
// BACKEND TYPE DEFINITIONS
// ============================================================================

struct TaskHandleImpl {
    TaskHandle_t freertosHandle;
    std::function<void()>* funcPtr; // Heap-allocated to persist beyond taskCreate
};

struct QueueHandleImpl {
    QueueHandle_t freertosHandle;
};

struct MutexHandleImpl {
    SemaphoreHandle_t freeRtosMutex;
};

struct SemaphoreHandleImpl {
    SemaphoreHandle_t freeRtosSemaphore;
};

} // namespace impl

// ============================================================================
// PRIORITY MAPPING
// ============================================================================

static UBaseType_t mapPriority(Priority prio) {
    // FreeRTOS: Higher number = higher priority
    // configMAX_PRIORITIES is typically 25 on ESP32
    switch (prio) {
        case Priority::IDLE:     return 0;
        case Priority::LOW:      return 2;
        case Priority::MEDIUM:   return 5;
        case Priority::HIGH:     return 10;
        case Priority::REALTIME: return 15;
        default:                 return 5;
    }
}

// ============================================================================
// TASK API IMPLEMENTATION
// ============================================================================

// Static task wrapper that calls the std::function
static void taskWrapper(void* param) {
    auto* impl = static_cast<impl::TaskHandleImpl*>(param);
    if (impl && impl->funcPtr) {
        (*impl->funcPtr)(); // Call the user's function
    }
    
    // Task function returned - clean up and delete self
    if (impl) {
        delete impl->funcPtr;
        delete impl;
    }
    vTaskDelete(NULL);
}

Result taskCreate(const TaskConfig& config, TaskHandle* outHandle) {
    if (!config.func) {
        return Result::INVALID;
    }
    
    // Allocate implementation structure
    auto* impl = new impl::TaskHandleImpl;
    impl->funcPtr = new std::function<void()>(config.func);
    
    BaseType_t result = xTaskCreate(
        taskWrapper,
        config.name,
        config.stackSize / sizeof(StackType_t), // Convert bytes to words
        impl,
        mapPriority(config.priority),
        &impl->freertosHandle
    );
    
    if (result != pdPASS) {
        delete impl->funcPtr;
        delete impl;
        ESP_LOGE(TAG, "Failed to create task: %s", config.name);
        return Result::NO_MEMORY;
    }
    
    if (outHandle) {
        *outHandle = TaskHandle(impl);
    }
    
    ESP_LOGI(TAG, "Created task: %s (stack: %zu, priority: %d)", 
             config.name, config.stackSize, mapPriority(config.priority));
    return Result::OK;
}

Result taskDelete(TaskHandle handle) {
    if (handle.getImpl() == nullptr) {
        // Delete current task
        vTaskDelete(NULL);
        return Result::OK;
    }
    
    auto* impl = handle.getImpl();
    if (impl->freertosHandle) {
        vTaskDelete(impl->freertosHandle);
        delete impl->funcPtr;
        delete impl;
    }
    
    return Result::OK;
}

void taskDelay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t getTickCount() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// ============================================================================
// QUEUE API IMPLEMENTATION
// ============================================================================

Result queueCreate(size_t itemSize, size_t capacity, QueueHandle* outHandle) {
    if (!outHandle || itemSize == 0 || capacity == 0) {
        return Result::INVALID;
    }
    
    QueueHandle_t freertosQueue = xQueueCreate(capacity, itemSize);
    if (freertosQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue (itemSize=%zu, capacity=%zu)", itemSize, capacity);
        return Result::NO_MEMORY;
    }
    
    auto* impl = new impl::QueueHandleImpl;
    impl->freertosHandle = freertosQueue;
    *outHandle = QueueHandle(impl);
    
    ESP_LOGI(TAG, "Created queue (itemSize=%zu, capacity=%zu)", itemSize, capacity);
    return Result::OK;
}

Result queueSend(QueueHandle handle, const void* item, uint32_t timeoutMs) {
    if (!handle.isValid() || !item) {
        return Result::INVALID;
    }
    
    auto* impl = handle.getImpl();
    TickType_t ticks = (timeoutMs == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    
    BaseType_t result = xQueueSend(impl->freertosHandle, item, ticks);
    return (result == pdTRUE) ? Result::OK : Result::TIMEOUT;
}

Result queueReceive(QueueHandle handle, void* item, uint32_t timeoutMs) {
    if (!handle.isValid() || !item) {
        return Result::INVALID;
    }
    
    auto* impl = handle.getImpl();
    TickType_t ticks = (timeoutMs == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    
    BaseType_t result = xQueueReceive(impl->freertosHandle, item, ticks);
    return (result == pdTRUE) ? Result::OK : Result::TIMEOUT;
}

Result queueOverwrite(QueueHandle handle, const void* item) {
    if (!handle.isValid() || !item) {
        return Result::INVALID;
    }
    
    auto* impl = handle.getImpl();
    BaseType_t result = xQueueOverwrite(impl->freertosHandle, item);
    return (result == pdTRUE) ? Result::OK : Result::ERROR;
}

size_t queueWaiting(QueueHandle handle) {
    if (!handle.isValid()) {
        return 0;
    }
    
    auto* impl = handle.getImpl();
    return uxQueueMessagesWaiting(impl->freertosHandle);
}

void queueDelete(QueueHandle handle) {
    if (handle.isValid()) {
        auto* impl = handle.getImpl();
        vQueueDelete(impl->freertosHandle);
        delete impl;
    }
}

// ============================================================================
// MUTEX API IMPLEMENTATION
// ============================================================================

Result mutexCreate(MutexHandle* outHandle) {
    if (!outHandle) {
        return Result::INVALID;
    }
    
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return Result::NO_MEMORY;
    }
    
    auto* impl = new impl::MutexHandleImpl;
    impl->freeRtosMutex = mutex;
    *outHandle = MutexHandle(impl);
    
    return Result::OK;
}

Result mutexLock(MutexHandle handle, uint32_t timeoutMs) {
    if (!handle.isValid()) {
        return Result::INVALID;
    }
    
    auto* impl = handle.getImpl();
    TickType_t ticks = (timeoutMs == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    
    BaseType_t result = xSemaphoreTake(impl->freeRtosMutex, ticks);
    return (result == pdTRUE) ? Result::OK : Result::TIMEOUT;
}

Result mutexUnlock(MutexHandle handle) {
    if (!handle.isValid()) {
        return Result::INVALID;
    }
    
    auto* impl = handle.getImpl();
    BaseType_t result = xSemaphoreGive(impl->freeRtosMutex);
    return (result == pdTRUE) ? Result::OK : Result::ERROR;
}

void mutexDelete(MutexHandle handle) {
    if (handle.isValid()) {
        auto* impl = handle.getImpl();
        vSemaphoreDelete(impl->freeRtosMutex);
        delete impl;
    }
}

// ============================================================================
// SEMAPHORE API IMPLEMENTATION
// ============================================================================

Result semaphoreCreateBinary(SemaphoreHandle* outHandle) {
    if (!outHandle) {
        return Result::INVALID;
    }
    
    SemaphoreHandle_t sem = xSemaphoreCreateBinary();
    if (sem == NULL) {
        ESP_LOGE(TAG, "Failed to create binary semaphore");
        return Result::NO_MEMORY;
    }
    
    auto* impl = new impl::SemaphoreHandleImpl;
    impl->freeRtosSemaphore = sem;
    *outHandle = SemaphoreHandle(impl);
    
    return Result::OK;
}

Result semaphoreTake(SemaphoreHandle handle, uint32_t timeoutMs) {
    if (!handle.isValid()) {
        return Result::INVALID;
    }
    
    auto* impl = handle.getImpl();
    TickType_t ticks = (timeoutMs == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    
    BaseType_t result = xSemaphoreTake(impl->freeRtosSemaphore, ticks);
    return (result == pdTRUE) ? Result::OK : Result::TIMEOUT;
}

Result semaphoreGive(SemaphoreHandle handle) {
    if (!handle.isValid()) {
        return Result::INVALID;
    }
    
    auto* impl = handle.getImpl();
    BaseType_t result = xSemaphoreGive(impl->freeRtosSemaphore);
    return (result == pdTRUE) ? Result::OK : Result::ERROR;
}

void semaphoreDelete(SemaphoreHandle handle) {
    if (handle.isValid()) {
        auto* impl = handle.getImpl();
        vSemaphoreDelete(impl->freeRtosSemaphore);
        delete impl;
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

[[noreturn]] void panic(const char* message) {
    ESP_LOGE(TAG, "PANIC: %s", message);
    esp_restart();
    while(1); // Should never reach here
}

void init() {
    ESP_LOGI(TAG, "RTOS abstraction layer initialized (FreeRTOS backend)");
}

} // namespace os
