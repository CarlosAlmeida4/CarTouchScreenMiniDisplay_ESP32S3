# Zephyr RTOS Implementation Changes Log

## Overview
This document tracks all architectural changes made to migrate the CarTouchScreenMiniDisplay project from FreeRTOS (ESP-IDF) to RTOS-agnostic design with Zephyr RTOS support.

**Project:** CarTouchScreenMiniDisplay_ESP32S3  
**Target Platform:** ESP32-S3  
**Current RTOS:** FreeRTOS (ESP-IDF)  
**Target RTOS:** Zephyr RTOS  
**Migration Strategy:** OS Abstraction Layer

---

## Phase 1: Discovery & Analysis (In Progress)

### 2026-07-22 - Initial RTOS Dependency Audit

#### Discovered FreeRTOS Dependencies

**System Component (`components/System/`)**
- Includes: `freertos/FreeRTOS.h`, `freertos/queue.h`, `freertos/task.h`
- Direct API usage:
  - `QueueHandle_t` - 2 instances (RollPitchQueue, WifiMgrQueue)
  - `xQueueCreate()` - constructor initialization
- Indirect dependencies: All sub-components use queues for inter-component communication

**qmi8658cInterface Component (`components/qmi8658cInterface/`)**
- Includes: `freertos/queue.h`
- Direct API usage:
  - `QueueHandle_t` - member variable Queue_
  - `xQueueOverwrite()` - sending sensor data
  - `vTaskDelay()` - 40ms sensor read loop delay
  - `vTaskDelete()` - error handling
- C++ synchronization:
  - `std::mutex` (2 instances) - RPOffsetMutex, PitchnRollMutex
  - `std::lock_guard` - RAII mutex locking
- Task pattern: Infinite loop with periodic delay (40ms)

**WifiManager Component (`components/WifiManager/`)**
- Includes: `freertos/FreeRTOS.h`, `freertos/task.h`
- Direct API usage:
  - `QueueHandle_t` - member variable Queue_
- ESP-IDF WiFi dependencies (not RTOS but platform-specific):
  - `esp_wifi.h`, `esp_event.h`
  - Network provisioning framework
- Thread-safe atomics: `std::atomic`

**Display Component (`components/display/`)**
- Includes: `freertos/queue.h`
- Direct API usage:
  - `QueueHandle_t` - 2 instances (RollPitchQueue_, WifiQueue_)
  - Likely queue read operations in implementation
- LVGL integration (runs in task context)

**main.cpp**
- Includes: `freertos/FreeRTOS.h`, `freertos/task.h`
- Entry point: `app_main()` - FreeRTOS convention

#### Summary of FreeRTOS Primitives Used
| Primitive | Count | Components | Purpose |
|-----------|-------|------------|---------|
| `QueueHandle_t` | ~6 | System, qmi8658c, Display, WifiMgr | Inter-task communication |
| `xQueueCreate()` | 2 | System | Queue initialization |
| `xQueueOverwrite()` | ≥1 | qmi8658c | Non-blocking data publish |
| `vTaskDelay()` | ≥1 | qmi8658c | Periodic timing |
| `vTaskDelete()` | ≥1 | qmi8658c | Error cleanup |
| `std::mutex` | 2 | qmi8658c | Data protection (not FreeRTOS, but RTOS-dependent) |

#### Non-RTOS Platform Dependencies Identified
- ESP-IDF HAL: GPIO, I2C, SPI, NVS, WiFi stack
- LVGL graphics library (RTOS-agnostic but needs timer integration)
- Network provisioning (ESP-IDF specific)

---

**Display Component (`components/display/`) - COMPLETE ANALYSIS**
- Includes: `freertos/queue.h`
- Direct API usage:
  - `QueueHandle_t` - 2 instances (RollPitchQueue_, WifiQueue_)
  - `xQueueReceive()` - Reads sensor/WiFi data
  - `xTaskCreate()` - Creates LVGL task (4KB stack, priority 2)
  - `SemaphoreHandle_t` - LVGL mutex for thread safety
  - `xSemaphoreGive()`, `xSemaphoreTake()` - LVGL locking
- Hidden dependencies:
  - `esp_timer` for periodic tick callbacks (LVGL tick source)
  - I2C/SPI drivers (touch, LCD) - ESP-IDF HAL dependency

**OTAUpdater Component (`components/OTAUpdater/`) - COMPLETE ANALYSIS**
- Includes: `freertos/FreeRTOS.h`, `freertos/task.h`
- Direct API usage:
  - `xTaskCreate()` - Creates OTA download task (8KB stack, priority 5)
  - `vTaskDelete()` - Self-deletes task on completion
- Major challenges:
  - ⚠️ **ESP-IDF OTA mechanism** (`esp_https_ota`, partition tables) - No direct Zephyr equivalent
  - Requires MCUboot porting for Zephyr OTA

**Diagnostics Component (`components/Diagnostics/`) - COMPLETE ANALYSIS**
- Includes: `freertos/task.h`
- Direct API usage:
  - `vTaskDelay()` - Yields to LVGL task (1ms delays in HTTP handlers)
  - `std::mutex` - Protects log buffer and server state (2 instances)
- Hidden dependencies:
  - `esp_http_server` - ESP-IDF HTTP stack (Zephyr uses different networking)
  - NVS flash (reset reason persistence)

#### Summary of ALL FreeRTOS Primitives Used
| Primitive | Count | Components | Purpose | Priority |
|-----------|-------|------------|---------|----------|
| `Task Create/Delete` | ~6 tasks | Display, QMI, OTA, WiFi | Task management | 🔴 Critical |
| `QueueHandle_t` | ~6 | System, qmi8658c, Display, WifiMgr | Inter-task comm | 🔴 Critical |
| `xQueueCreate()` | 2 | System | Queue initialization | 🔴 Critical |
| `xQueueOverwrite()` | 1 | qmi8658c | Non-blocking publish | 🔴 Critical |
| `xQueueReceive()` | Multiple | Display | Data consumption | 🔴 Critical |
| `SemaphoreHandle_t` | 1 | Display (LVGL) | Binary semaphore | 🟡 Medium |
| `vTaskDelay()` | Frequent | qmi8658c, Diagnostics, WiFi | Periodic timing | 🔴 Critical |
| `vTaskDelete()` | 2 | qmi8658c, OTA | Error handling | 🟢 Low |
| `std::mutex` | ~8 | qmi8658c, Display, Diagnostics, WiFi | Data protection | 🔴 Critical |
| `std::atomic` | 2 | WiFi | Lock-free flags | 🟢 Low |

---

## Phase 2: Design Decisions

### 2026-07-22 - OS Abstraction Layer Architecture

#### Key Design Decisions

| **Aspect** | **Decision** | **Rationale** |
|------------|-------------|---------------|
| **Queue Type Safety** | Type-erased base + template wrappers | Balances Zephyr compile-time size with C++ type safety |
| **Task Stack Sizing** | Component header constants | Clear documentation, easy tuning |
| **Error Handling** | Return codes + CHECK macros | Matches ESP-IDF patterns, explicit handling |
| **Scheduling Priorities** | Abstract levels (HIGH/MEDIUM/LOW) | Shields from FreeRTOS↔Zephyr inversion |
| **Build System** | Separate (ESP-IDF + future Zephyr) | Phased migration, maintains baseline |
| **Per-Component Abstraction** | Component-local OS headers | Reduces coupling, easier testing |

#### Abstraction Layer Structure

```
os_abstraction/
├── os_primitives.h       # Core RTOS API (tasks, queues, mutexes, delays)
├── freertos_backend.cpp  # FreeRTOS implementation
└── zephyr_backend.cpp    # Zephyr implementation (future)

components/<component>/
└── <component>_os.h      # Component-specific OS primitives
```

#### API Design Philosophy

1. **Minimal Surface Area:** Only abstract what components actually use
2. **Type Safety:** Use templates where possible, type-erasure where necessary
3. **RAII Patterns:** Lock guards, queue handles with destructors
4. **Explicit Errors:** Return codes, not exceptions (embedded-friendly)
5. **Zero Overhead:** Inline wrappers, compile-time dispatch

---

## Phase 3: Implementation Plan

### Step 1: Core Abstraction Layer ✅ COMPLETE
- [x] Create `os_abstraction/os_primitives.h` with:
  - Task creation/deletion API (C++ std::function-based)
  - Queue create/send/receive/overwrite API (template-based type safety)
  - Mutex/Semaphore API (RAII MutexGuard)
  - Delay API (millisecond precision)
  - Priority abstraction (IDLE/LOW/MEDIUM/HIGH/REALTIME)
- [x] Implement `os_abstraction/freertos_backend.cpp`
- [x] Add CMakeLists.txt for os_abstraction component

**Files Created:**
- `components/os_abstraction/os_primitives.h` (342 lines)
- `components/os_abstraction/freertos_backend.cpp` (168 lines)
- `components/os_abstraction/CMakeLists.txt`

**Key Features Implemented:**
- Template `os::Queue<T>` wrapper for compile-time type safety
- Priority mapping (shields from FreeRTOS vs Zephyr priority inversion)
- RAII `MutexGuard` for automatic unlock
- Error handling via `os::Result` enum (OK, TIMEOUT, INVALID_PARAM, ERROR)

### Step 2: Per-Component OS Headers ✅ COMPLETE
- [x] `components/System/system_os.h` - Queue creation wrappers
- [x] `components/display/display_os.h` - Task + Queue + LVGL mutex (binary semaphore)
- [x] `components/qmi8658cInterface/qmi_os.h` - Sensor task + Queue overwrite + Delays
- [x] `components/OTAUpdater/ota_os.h` - OTA task creation/deletion
- [x] `components/Diagnostics/diag_os.h` - HTTP handler cooperative yields
- [x] `components/WifiManager/wifi_os.h` - WiFi manager + provisioning tasks

**Files Created:**
- `components/System/system_os.h` (84 lines)
- `components/display/display_os.h` (112 lines)
- `components/qmi8658cInterface/qmi_os.h` (93 lines)
- `components/OTAUpdater/ota_os.h` (66 lines)
- `components/Diagnostics/diag_os.h` (64 lines)
- `components/WifiManager/wifi_os.h` (101 lines)

**Component-Specific Abstractions:**
- **System:** Queue factory functions for RollPitch and WiFiManager pipelines
- **Display:** LVGL mutex class, task creation, queue polling with timeouts
- **QMI8658c:** High-priority sensor task, queue overwrite for latest data, self-deletion on error
- **OTA:** 8KB stack task for HTTPS downloads, self-deletion on completion
- **Diagnostics:** Cooperative multitasking delays to prevent LVGL watchdog timeout
- **WiFi:** Manager and provisioning task wrappers

### Step 3: Refactor Components (FreeRTOS Backend) 🔄 NOT STARTED
- [ ] System: Replace `xQueueCreate` with `SystemOS::createRollPitchQueue()`
- [ ] Display: Replace task/queue/semaphore APIs with `DisplayOS` wrappers
- [ ] qmi8658c: Replace task/queue/delay/delete APIs with `QMIOS` wrappers
- [ ] OTA: Replace task APIs with `OTAOS` wrappers
- [ ] Diagnostics: Replace `vTaskDelay(1)` with `DiagOS::yieldToLVGL()`
- [ ] WiFi: Replace task/delay APIs with `WifiOS` wrappers

**Status:** Infrastructure complete, refactoring not started yet

### Step 4: Testing & Validation ⏱️ BLOCKED
- [ ] Build with FreeRTOS backend (will fail until Step 3 complete)
- [ ] Test all tasks start correctly
- [ ] Test queue communication (sensor, WiFi)
- [ ] Test LVGL rendering + touch
- [ ] Test WiFi + OTA functionality
- [ ] Test diagnostics HTTP server

**Blocker:** Components still use raw FreeRTOS APIs. Must complete Step 3 first.

### Step 5: Future Zephyr Backend ⏱️ NOT STARTED
- [ ] Implement `os_abstraction/zephyr_backend.cpp`
- [ ] Port SPI/I2C drivers
- [ ] Port WiFi to Zephyr networking
- [ ] Port OTA to MCUboot
- [ ] Create Kconfig + devicetree files

---

## Phase 3: Implementation Details

### 2026-07-22 - OS Abstraction Infrastructure Created

#### Core Abstraction API (`os_primitives.h`)

**Task API:**
```cpp
struct TaskConfig {
    const char* name;
    os::Priority priority;
    size_t stackSize;
    uint32_t coreAffinity;
};

os::TaskHandle taskCreate(const TaskConfig& config, std::function<void()> taskFunction);
void taskDelete(os::TaskHandle handle);
void taskDelay(uint32_t milliseconds);
```

**Queue API (Type-Safe Template):**
```cpp
template<typename T>
class Queue {
    os::QueueHandle handle_;
public:
    Result send(const T& item, uint32_t timeoutMs);
    Result receive(T& item, uint32_t timeoutMs);
    Result overwrite(const T& item);  // Non-blocking
};
```

**Mutex API (RAII):**
```cpp
os::MutexHandle mutexCreate();
class MutexGuard {
    // RAII lock/unlock
};
```

**Semaphore API:**
```cpp
os::SemaphoreHandle semaphoreCreate(uint32_t maxCount, uint32_t initialCount);
Result semaphoreTake(os::SemaphoreHandle handle, uint32_t timeoutMs);
Result semaphoreGive(os::SemaphoreHandle handle);
```

#### FreeRTOS Backend Implementation

**Priority Mapping:**
```cpp
// FreeRTOS: higher number = higher priority
static uint32_t mapPriority(os::Priority prio) {
    switch (prio) {
        case os::Priority::IDLE:     return 0;   // tskIDLE_PRIORITY
        case os::Priority::LOW:      return 2;
        case os::Priority::MEDIUM:   return 5;
        case os::Priority::HIGH:     return 10;
        case os::Priority::REALTIME: return configMAX_PRIORITIES - 1;
    }
}
```

**Task Wrapper (std::function support):**
```cpp
static void taskWrapperFunction(void* pvParameters) {
    auto* taskFunc = static_cast<std::function<void()>*>(pvParameters);
    (*taskFunc)();
    delete taskFunc;
}

os::TaskHandle taskCreate(const TaskConfig& config, std::function<void()> taskFunction) {
    auto* heapFunc = new std::function<void()>(std::move(taskFunction));
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(taskWrapperFunction, config.name, config.stackSize,
                            heapFunc, mapPriority(config.priority), &handle, config.coreAffinity);
    return reinterpret_cast<os::TaskHandle>(handle);
}
```

**Queue Overwrite (Critical for Sensor Data):**
```cpp
Result queueOverwrite(QueueHandle handle, const void* item) {
    auto xStatus = xQueueOverwrite(reinterpret_cast<QueueHandle_t>(handle), item);
    return (xStatus == pdPASS) ? Result::OK : Result::ERROR;
}
```

#### Component OS Headers - Design Patterns

**1. SystemOS - Factory Functions**
```cpp
namespace SystemOS {
    inline os::Queue<PipelineTypes::RollPitch> createRollPitchQueue() {
        return os::Queue<PipelineTypes::RollPitch>(1);
    }
    inline os::Queue<PipelineTypes::WifiManagerPipelineModes> createWifiManagerQueue() {
        return os::Queue<PipelineTypes::WifiManagerPipelineModes>(1);
    }
}
```

**2. DisplayOS - LVGL Thread Safety**
```cpp
class LVGLMutex {
    os::SemaphoreHandle semaphore_;
public:
    LVGLMutex();
    void lock() { os::semaphoreTake(semaphore_, os::WAIT_FOREVER); }
    void unlock() { os::semaphoreGive(semaphore_); }
};
```

**3. QMIOS - Sensor Task Wrapper**
```cpp
namespace QMIOS {
    template<typename TaskFn>
    os::TaskHandle createSensorTask(TaskFn&& taskFn) {
        return os::taskCreate({
            .name = "QMI8658_Task",
            .priority = os::Priority::HIGH,  // Time-critical sensor reading
            .stackSize = 4096,
            .coreAffinity = tskNO_AFFINITY
        }, std::forward<TaskFn>(taskFn));
    }
}
```

**4. OTAOS - Long-Running Download Task**
```cpp
namespace OTAOS {
    template<typename TaskFn>
    os::TaskHandle createOTATask(TaskFn&& taskFn) {
        return os::taskCreate({
            .name = "OTA_Task",
            .priority = os::Priority::MEDIUM,
            .stackSize = 8192,  // HTTPS/TLS needs more stack
            .coreAffinity = tskNO_AFFINITY
        }, std::forward<TaskFn>(taskFn));
    }
}
```

**5. DiagOS - HTTP Handler Yielding**
```cpp
namespace DiagOS {
    inline void yieldToLVGL() {
        os::taskDelay(1);  // 1ms cooperative yield
    }
}
```

---

---

## Phase 4: Zephyr Integration (Not Started)

_Zephyr-specific configuration and porting notes will be documented here._

---

## Lessons Learned

_Architectural insights and gotchas discovered during migration will be documented here._

---

**Last Updated:** 2026-07-22
