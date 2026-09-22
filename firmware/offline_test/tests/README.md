# Offline Unit & Integration Tests (`offline_test/tests/`)

This directory contains automated unit and integration tests for the simulation engine and simulated firmware components. All tests are integrated with CMake and CTest.

---

## 1. Building and Running Tests

### Using CMake Presets (Recommended)
From `firmware/offline_test`:

```bash
# 1. Configure the build with vcpkg integration
cmake --preset default

# 2. Build all test targets
cmake --build build

# 3. Execute all tests via CTest
ctest --test-dir build --output-on-failure
```

### Running Individual Test Binaries
You can run any test binary directly from the build directory to see detailed console output:

```bash
./build/tests/test_virtual_clock
```

---

## 2. Existing Test Suites

### `test_virtual_clock.cpp`
Source: [`test_virtual_clock.cpp`](test_virtual_clock.cpp)

Validates the core simulation timing and scheduling engine, as well as 32-bit binary layout:

0. **Architecture Checks**:
   Enforces at compile time that `sizeof(void*) == 4` and `sizeof(size_t) == 4`, guaranteeing 32-bit pointer and integer alignment parity with target microcontroller hardware.
1. **`test_clock_jump()`**:
   Launches a fiber that requests a 3,000,000 microsecond (3.0 second) virtual delay. Verifies that virtual time jumps forward instantly and executes in under 10 milliseconds of host wall-clock time.
2. **`test_priority_scheduling()`**:
   Schedules two fibers for the exact same wake-up deadline: one with `PRIO_FIRMWARE` (10) and one with `PRIO_ACTUATOR_PHYSICS` (0). Verifies that the priority scheduler strictly awakens the high-priority fiber first.
3. **`test_interleaved_delays()`**:
   Launches two concurrent fibers interleaving multiple delays (`100 us`, `150 us`, `300 us`, `350 us`). Verifies that event dispatch maintains chronological order without race conditions.

### `test_uart_bus.cpp`
Source: [`test_uart_bus.cpp`](test_uart_bus.cpp)

Validates the full serial communication stack:

1. **`test_firmware_to_bus_write()`**:
   Firmware `Uart::write()` and `print()` calls send data to `UartBus`, which queues bytes for simulated peripherals.
2. **`test_bus_to_firmware_read()`**:
   Peripheral/harness `UartBus::write_to_firmware()` pushes data received by firmware `Uart::available()`, `peek()`, and `read()`.
3. **`test_sim_peeking()`**:
   Verifies that `bus->peek_fw_tx()` and `bus->peek_all_fw_tx()` allow the simulator to inspect pending buffer contents non-destructively without popping data.
4. **`test_baud_rate_timing()`**:
   Transmits 10 bytes at 9600 baud and asserts that exactly 10,416 virtual microseconds advance during transmission.
5. **`test_snooper_formatting()`**:
   Validates formatted string output for `FORMAT_ASCII`, `FORMAT_HEX`, and `FORMAT_HEX_DUMP` modes.
6. **`test_terminal_harness_injection()`**:
   Injects console input through `UartTerminalHarness` and verifies receipt by firmware.

### `test_rs485_mux.cpp`
Source: [`test_rs485_mux.cpp`](test_rs485_mux.cpp)

Validates GPIO state tracking, RS-485 multiplexing, bus collision detection, pluggable model substitution, and AMT242AV encoder emulation:

1. **`test_gpio_tracking_and_listeners()`**:
   Tests `pinMode()`, `digitalWrite()`, `digitalRead()`, and edge callback triggering on pin state transitions.
2. **`test_rs485_mux_single_device()`**:
   Tests that UART traffic is completely blocked when `SEL` is `LOW` and routed to the peripheral when `SEL` is `HIGH`. Also asserts that unselected devices cannot drive the bus.
3. **`test_multiple_devices_muxing_isolation()`**:
   Attaches two devices to the same `RS485Mux` and validates strict bus isolation as `SEL` lines are toggled.
4. **`test_bus_contention_detection()`**:
   Asserts two `SEL` lines `HIGH` simultaneously. Verifies that `has_bus_contention()` triggers, bus traffic is dropped/quarantined, and normal operation resumes once contention is resolved.
5. **`test_pluggable_substitution()`**:
   Demonstrates how a developer can plug in a custom motor controller model using `FunctionalRS485Device` in 3 lines of code.
6. **`test_amt242av_encoder_model()`**:
   Validates 12-bit position formatting, 2-bit odd parity checksum calculation, and zero/reset command execution for the `AMT242AV_Sim` model.
7. **`test_firmware_uart_class_integration()`**:
   Executes the exact firmware sequence from `AMT242AV::_read_pos()` via the `Uart` class and verifies reading 12-bit position back into firmware.

---

## 3. Writing a New Test

When authoring a new test suite:
1. Always call `toad::sim::install_fiber_scheduler()` on the main test thread before launching fibers.
2. Reset the clock via `VirtualClock::instance().reset(0)`.
3. Launch fibers using `launch_fiber_with_priority(prio, ...)`.
4. Advance the simulation using `VirtualClock::instance().run_until(target_us)`.
5. Join all fibers before asserting post-conditions.

### Example Template
```cpp
#include <cassert>
#include <iostream>
#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"
#include "hal_mock/Arduino.h"

using namespace toad::sim;

void test_my_feature() {
    std::cout << "[Test] Testing my feature..." << std::endl;
    VirtualClock::instance().reset(0);

    bool completed = false;
    auto f = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        delay(50); // 50 ms delay
        completed = true;
    });

    VirtualClock::instance().run_until(50000); // 50,000 us
    f.join();

    assert(completed);
    assert(millis() == 50);
    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    install_fiber_scheduler();
    test_my_feature();
    return 0;
}
```

### Adding New Test to `CMakeLists.txt`
In `offline_test/CMakeLists.txt`:
```cmake
add_executable(test_my_feature tests/test_my_feature.cpp)
target_link_libraries(test_my_feature PRIVATE toad_sim_core toad_hal_mock)
add_test(NAME test_my_feature COMMAND test_my_feature)
```

