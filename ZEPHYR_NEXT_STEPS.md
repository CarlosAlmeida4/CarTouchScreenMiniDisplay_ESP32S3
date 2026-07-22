# Zephyr RTOS Migration - Next Steps Guide

## Overview
This guide documents the steps required to complete the migration from ESP-IDF/FreeRTOS to Zephyr RTOS for the ESP32-S3 touchscreen display project.

**Current Status:** Phase 1 (Discovery) in progress  
**Completion:** 0% (Initial analysis phase)

---

## Phase 1: Discovery & RTOS Abstraction Design (Current)

### Step 1: Complete RTOS Dependency Audit ⏳
**Status:** In Progress

**What you need to do:**
1. Analyze remaining components not yet audited:
   - `components/OTAUpdater/` - Check for tasks, timers, HTTP client threading
   - `components/Diagnostics/` - Check for HTTP server threads, logging context
   - `components/SensorLib/` - Check for any blocking I/O or timing dependencies

2. For each component, identify:
   - Task creation patterns (what threads exist, their priorities, stack sizes)
   - Synchronization primitives (mutexes, semaphores, event groups)
   - Inter-task communication (queues, pipes, message passing)
   - Timing operations (delays, timeouts, periodic timers)
   - ISR/callback contexts (what runs in interrupt vs task context)

3. Document in `ZEPHYR_IMPLEMENTATION_CHANGES.md` under Phase 1

**Tools to help:**
- Search for includes: `#include "freertos/`
- Search for API calls: `xQueue`, `xTask`, `xSemaphore`, `xTimer`, `vTask`, `port`
- Check for C++ threading: `std::thread`, `std::mutex`, `std::condition_variable`

---

### Step 2: Define Abstraction Layer Requirements ⏱️
**Status:** Not Started  
**Depends on:** Step 1 completion

**What you need to do:**
1. Based on audit, determine required OS primitives:
   - Thread/Task abstraction
   - Queue abstraction (message passing)
   - Mutex abstraction (mutual exclusion)
   - Semaphore abstraction (if used)
   - Timer abstraction (periodic/one-shot)
   - Delay/sleep abstraction
   - Memory allocation abstraction (heap, static, pool)
   - Atomic operations (if beyond std::atomic)

2. Decide on abstraction design pattern:
   - **Option A:** C++ class-based wrapper (modern, type-safe, RAII-friendly)
   - **Option B:** C function pointers with backends (portable, C-compatible)
   - **Option C:** Header-only template layer (zero overhead, complex)
   - **Recommendation:** Option A for your C++ codebase

3. Answer these design questions:
   - Do you want compile-time (Kconfig) or runtime OS selection?
   - Should the abstraction be header-only or compiled library?
   - How will error handling work across RTOS boundaries?
   - Will you support static allocation (Zephyr native) or dynamic only?

---

### Step 3: Design Abstraction API Interface ⏱️
**Status:** Not Started  
**Depends on:** Step 2 completion

**What you need to design:**

A component-local abstraction layer with these characteristics:
- **Location:** Each component gets its own `<component>_os.hpp/cpp` (e.g., `qmi8658c_os.hpp`)
- **Scope:** Contains ONLY the OS primitives that specific component needs
- **Backend:** Implementation files that map to FreeRTOS initially, Zephyr later

**Example Interface Concepts (Not Full Code):**

```cpp
// File: components/qmi8658cInterface/qmi8658c_os.hpp
namespace qmi8658c_os {
    // Queue for RollPitch data
    class MessageQueue { /* ... */ };
    
    // Mutex for offset protection
    class Mutex { /* ... */ };
    
    // Delay for sensor loop
    void delay_ms(uint32_t ms);
    
    // Task/thread spawning
    void create_sensor_task(/* params */);
}
```

**Think about:**
- What's the minimal API surface each component needs?
- How do you handle queue types (type-safe vs void*)?
- Should mutexes be recursive? Timeout support?
- Stack size/priority configuration for Zephyr vs FreeRTOS?

---

## Phase 2: FreeRTOS Backend Implementation (Future)

### Step 4: Implement FreeRTOS Backend ⏱️
**Status:** Not Started  
**Depends on:** Step 3 completion

**What you need to do:**
1. Create `*_os.cpp` files that map your abstraction to FreeRTOS APIs
2. Keep existing behavior exactly the same (no functional changes)
3. Update components to use abstraction layer instead of direct FreeRTOS calls
4. Test thoroughly to ensure no regressions

**Validation criteria:**
- All existing features work identically
- Build system compiles cleanly
- No performance degradation
- Code is cleaner and more testable

---

## Phase 3: Zephyr Backend Preparation (Future)

### Step 5: Create Zephyr Project Structure ⏱️
**Status:** Not Started  
**Depends on:** Step 4 completion

**What you need to create:**
1. **Zephyr west workspace:**
   ```
   zephyr-project/
   ├── zephyr/            (Zephyr SDK)
   ├── modules/           (external modules)
   └── app/               (your application)
       ├── prj.conf       (Kconfig configuration)
       ├── boards/
       │   └── esp32s3_devkitc.overlay  (devicetree)
       ├── src/           (ported application code)
       └── CMakeLists.txt
   ```

2. **Devicetree overlay for your board:**
   - Define SPI bus for LCD
   - Define I2C bus for IMU and touch
   - Define GPIOs for control signals
   - Define partitions for NVS, OTA

3. **prj.conf configuration:**
   - Enable required drivers: SPI, I2C, GPIO, WiFi, BLE
   - Enable LVGL support
   - Enable network stack (TCP/IP, HTTP, MQTT if needed)
   - Enable NVS/settings for persistent storage
   - Configure thread stack sizes and priorities

**Key Zephyr concepts to learn:**
- Devicetree (hardware description)
- Kconfig (build configuration)
- Driver model (SPI, I2C, GPIO APIs)
- Networking stack (net_if, net_pkt, sockets)
- Settings subsystem (NVS replacement)

---

### Step 6: Implement Zephyr Backend ⏱️
**Status:** Not Started  
**Depends on:** Step 5 completion

**What you need to do:**
1. Create Zephyr versions of `*_os.cpp` files
2. Map abstractions to Zephyr kernel APIs:
   - Tasks → `k_thread` / `K_THREAD_DEFINE`
   - Queues → `k_msgq` or `k_fifo`
   - Mutexes → `k_mutex`
   - Delays → `k_sleep` / `k_msleep`
   - Timers → `k_timer`

3. Handle ESP-IDF specific code:
   - Replace `esp_wifi.h` with Zephyr WiFi APIs
   - Replace NVS with Zephyr Settings or NVS Flash
   - Replace ESP-IDF GPIO/SPI/I2C with Zephyr driver APIs
   - Port OTA implementation to MCUboot (Zephyr bootloader)

**Zephyr API Hints (Not Full Implementation):**
```cpp
// FreeRTOS queue → Zephyr message queue
K_MSGQ_DEFINE(my_msgq, sizeof(RollPitch), 1, 4);
k_msgq_put(&my_msgq, &data, K_NO_WAIT);

// FreeRTOS task → Zephyr thread
K_THREAD_DEFINE(sensor_thread, STACK_SIZE, sensor_task_entry,
                NULL, NULL, NULL, PRIORITY, 0, 0);

// FreeRTOS mutex → Zephyr mutex
K_MUTEX_DEFINE(my_mutex);
k_mutex_lock(&my_mutex, K_FOREVER);
```

---

### Step 7: Port ESP-IDF HAL Dependencies ⏱️
**Status:** Not Started  
**Depends on:** Step 6 completion

**Critical porting tasks:**

1. **WiFi Stack:**
   - ESP-IDF: `esp_wifi_*` APIs
   - Zephyr: `net_if`, `net_mgmt`, `wifi_mgmt.h`
   - Challenge: Provisioning framework has no direct Zephyr equivalent

2. **NVS (Non-Volatile Storage):**
   - ESP-IDF: `nvs_flash_*`
   - Zephyr: `settings` subsystem or `nvs` API (available for ESP32)

3. **OTA Updates:**
   - ESP-IDF: `esp_ota_*` APIs with custom partition scheme
   - Zephyr: MCUboot (bootloader) + DFU (Device Firmware Update)
   - Requires: Partition manager, image signing

4. **Driver APIs:**
   - ESP-IDF: `driver/spi_master.h`, `driver/i2c.h`, `driver/gpio.h`
   - Zephyr: `drivers/spi.h`, `drivers/i2c.h`, `drivers/gpio.h`
   - API surface is similar but not identical

5. **LVGL Integration:**
   - Both RTOSes support LVGL
   - Zephyr has native LVGL module
   - Needs: Display driver, input device driver, tick source

**Risk areas:**
- Network provisioning (may need custom implementation)
- Touch controller driver (CST92xx) - may need porting
- LCD driver (SH8601) - may need porting
- WiFi credential storage and management

---

## Phase 4: Testing & Validation (Future)

### Step 8: Incremental Testing Strategy ⏱️
**Status:** Not Started

**Testing approach:**
1. **Unit tests per component** (abstraction layer)
2. **Integration tests** (component interactions)
3. **Hardware-in-loop tests** (ESP32-S3 target)

**Test scenarios:**
- Display rendering at 60 FPS
- IMU data at 25 Hz to display pipeline
- WiFi provisioning and connection
- OTA update cycle
- Touch input responsiveness
- Multi-component stress test

---

## Critical Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| WiFi provisioning has no Zephyr equivalent | High | Implement custom BLE or HTTP-based provisioning |
| LCD/Touch drivers not in Zephyr mainline | Medium | Port drivers to Zephyr driver model or use out-of-tree |
| OTA requires MCUboot understanding | Medium | Study MCUboot documentation, use Zephyr samples |
| Performance regression in LVGL rendering | High | Profile both RTOSes, optimize Zephyr config |
| Different threading models cause deadlocks | High | Careful priority mapping, extensive testing |

---

## Resources You'll Need

### Documentation
- [Zephyr RTOS Documentation](https://docs.zephyrproject.org)
- [Zephyr ESP32 Board Support](https://docs.zephyrproject.org/latest/boards/xtensa/esp32s3_devkitc/doc/index.html)
- [Zephyr Kernel Services](https://docs.zephyrproject.org/latest/kernel/services/index.html)
- [MCUboot Documentation](https://docs.mcuboot.com)
- [LVGL on Zephyr](https://docs.zephyrproject.org/latest/samples/modules/lvgl/demos/index.html)

### Tools
- West (Zephyr build/flash tool)
- Devicetree compiler (dtc)
- OpenOCD or Segger J-Link (debugging)
- Logic analyzer (for bus debugging)

### Sample Projects
- `zephyr/samples/net/wifi` - WiFi examples
- `zephyr/samples/modules/lvgl` - LVGL examples
- `zephyr/samples/subsys/mgmt/mcumgr` - OTA examples

---

## Decision Log

### Design Decisions to Make

1. **Abstraction granularity:**
   - [ ] One abstraction library for all components vs per-component
   - [ ] Recommendation: Per-component (simpler, less coupling)

2. **Build system integration:**
   - [ ] Dual build support (ESP-IDF + Zephyr) vs migration flag day
   - [ ] Recommendation: Dual build during transition

3. **Static vs dynamic allocation:**
   - [ ] Zephyr prefers static (K_THREAD_DEFINE), FreeRTOS uses dynamic
   - [ ] Recommendation: Support both, use static where possible

4. **Priority mapping:**
   - [ ] FreeRTOS priorities (0=lowest) vs Zephyr priorities (0=highest, negative=preemptive)
   - [ ] Need explicit priority mapping table

5. **Queue semantics:**
   - [ ] FreeRTOS queues are copy-based, Zephyr has msgq (copy) and fifo (pointer)
   - [ ] Recommendation: Use msgq for data, fifo for event notifications

---

## Current Blockers

**None yet** - Still in discovery phase.

---

## Next Immediate Action

👉 **Complete Step 1:** Audit remaining components (OTAUpdater, Diagnostics, SensorLib)

See detailed instructions in Step 1 above.

---

**Last Updated:** 2026-07-22
