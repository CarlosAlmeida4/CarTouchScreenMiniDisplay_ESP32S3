# RTOS Abstraction Layer - Implementation TODO

## Status: Phase 1 - Discovery & Analysis

### Current Phase Tasks
- [ ] Identify all RTOS dependencies across components
- [ ] Document current FreeRTOS API usage patterns
- [ ] Design OS abstraction layer interface
- [ ] Create component-level abstraction headers

### Components to Process
- [ ] System - orchestration, queues
- [ ] qmi8658cInterface - tasks, delays, queues, mutexes
- [ ] WifiManager - tasks, events, callbacks
- [ ] Display - tasks, queues, LVGL timer integration
- [ ] OTAUpdater - (to be analyzed)
- [ ] Diagnostics - (to be analyzed)
- [ ] SensorLib - (to be analyzed)

### Phase 2 - Design (Not Yet Started)
- [ ] Define abstraction layer structure
- [ ] Create interface specifications
- [ ] Design build system integration

### Phase 3 - Implementation (Not Yet Started)
- [ ] Implement FreeRTOS backend
- [ ] Prepare Zephyr backend stubs
- [ ] Refactor components one-by-one

### Phase 4 - Zephyr Integration (Not Yet Started)
- [ ] Create Zephyr devicetree overlays
- [ ] Configure Zephyr prj.conf
- [ ] Port ESP-IDF specific code to Zephyr APIs
- [ ] Test and validate

---
**Last Updated:** 2026-07-22
