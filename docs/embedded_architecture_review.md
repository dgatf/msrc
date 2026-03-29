# MSRC Embedded Architecture Review

This review focuses on system-level behavior for the RP2040 + FreeRTOS firmware under real hardware load.

## Strengths worth preserving

- **Clear top-level split between transport, protocol, and sensor tasks.** `main.c` dispatches one receiver protocol task, while each protocol module owns its own `set_config()` and sensor/task wiring. That keeps most protocol-specific behavior localized.
- **Good use of RP2040 PIO for timing-sensitive serial paths.** The PIO-backed UART and capture helpers reduce CPU jitter on GPS, PWM capture, WS2812, and other edge-sensitive paths.
- **Simple sensor data flow.** Most sensor tasks publish through dedicated `float *` outputs and protocol tasks read those values directly, which avoids large message copies in steady state.
- **ISR-to-task notification model is lightweight.** UART paths primarily use queues for bytes and task notifications for packet completion, which is a good fit for short ISR work.

## Critical findings

### 1. ISR code used non-ISR-safe queue reset APIs

- **Files:** `board/project/uart.c`, `board/project/uart_pio.c`
- **Risk:** `xQueueReset()` was called from UART and PIO interrupt handlers whenever a timeout boundary was detected. That API is not safe in ISR context, so packet framing under load could corrupt queue state or trip undefined kernel behavior.
- **Mitigation implemented:** queue flushing now uses `uxQueueMessagesWaitingFromISR()` + `xQueueReceiveFromISR()` instead of `xQueueReset()`.

### 2. PIO UART initialization recreated the RX queue when TX was enabled

- **File:** `board/project/uart_pio.c`
- **Risk:** `uart_pio_begin()` allocated `context.uart_rx_pio_queue_handle` once for RX and then allocated it again when TX was also configured. GPS uses both TX and RX, so the second allocation replaced the first queue handle and leaked the original queue.
- **Mitigation implemented:** TX initialization no longer recreates the RX queue.

## Important findings

### 3. Runtime fault observability was disabled

- **Files:** `board/freertos/FreeRTOSConfig.h`, `board/project/main.c`
- **Risk:** stack overflow checking and malloc-failure hooks were disabled even though the system creates many small tasks and dynamically allocates sensor output storage during protocol setup. Failures would tend to become silent corruption or hard hangs instead of diagnosable faults.
- **Mitigation implemented:** stack overflow checking is enabled and a malloc-failure hook now halts with an explicit message.

### 4. SmartPort setup scales poorly because it mixes protocol registration with sensor instantiation

- **File:** `board/project/protocol/smartport.c`
- **Risk:** `set_config()` both interprets configuration and creates a large tree of sensor tasks, telemetry helper tasks, and heap-backed value pointers. The pattern works today, but adding new sensors or alternate lifetime rules means touching one very large function with duplicated allocation/wiring logic.
- **Improvement:** factor repeated “allocate sensor outputs + spawn producer + spawn telemetry publisher” sequences behind protocol-local helper functions or static tables so the transport logic stays separate from wiring logic.

### 5. Dynamic allocation has no local failure handling in protocol setup

- **Files:** `board/project/protocol/smartport.c` and similar protocol `set_config()` functions
- **Risk:** many telemetry producers allocate individual `float`, `double`, `uint8_t`, and `uint16_t` outputs with no per-allocation NULL checks. The new malloc-failure hook improves diagnosability, but not graceful degradation.
- **Improvement:** introduce small protocol-local allocation helpers that either fail the entire sensor registration cleanly or fall back to disabling only that sensor path.

### 6. Several protocols rely on implicit timing contracts rather than explicit state machines

- **Files:** especially `board/project/protocol/smartport.c`, `board/project/sensor/smart_esc.c`, `board/project/uart.c`
- **Risk:** timeout-driven framing, half-duplex turnarounds, and polling windows are distributed across alarms, queues, and task notifications. This is efficient, but it makes correctness depend on hidden timing assumptions such as “a packet boundary always corresponds to a queue flush” or “response work will complete before the next poll.”
- **Improvement:** document those contracts near the affected modules and prefer explicit per-link states when adding new protocols.

## Optional findings

### 7. Debug transport is efficient but weakly protected

- **Files:** `board/project/dbg_rb.c`, `board/project/dbg_task.c`
- **Risk:** the ring-buffer design is appropriate for low-overhead logging, but head/tail movement is not guarded. If debug writes ever expand into mixed-context producers, corruption will be difficult to diagnose.
- **Improvement:** keep the single-producer assumption documented, or add minimal interrupt masking around index updates if ISR logging is expected.

### 8. Queue and packet length types are narrower than the configured buffers

- **Files:** `board/project/uart.c`, `board/project/uart_pio.c`
- **Risk:** UART queues are sized to 512 bytes, but helper APIs report availability and lengths as `uint8_t`. Real telemetry frames are short enough today, yet the interface itself cannot represent the configured capacity and bakes in a silent truncation ceiling.
- **Improvement:** use `size_t` or `uint16_t` for queue depth and bulk read lengths.

## Architecture summary

The repository has a solid embedded foundation: modular protocol tasks, low-copy shared sensor values, and strong use of RP2040 hardware features. The main risks are not stylistic—they are the kind that surface after long runtimes or during bad line conditions: unsafe ISR API usage, hidden allocation failures, and timing assumptions spread across multiple layers. The fixes in this change address the two most immediate RTOS/kernel hazards and improve visibility into memory and stack failures.
