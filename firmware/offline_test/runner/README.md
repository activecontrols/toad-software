# Test Harness & Execution Runner (`offline_test/runner/`)

This directory houses the top-level orchestration entrypoint (`main.cpp`) for running the full Engine Controller Software-in-the-Loop (SITL) simulation.

---

## 1. Simulation Lifecycle

The runner orchestrates the simulation across five phases:

```
+------------------------------------------------------------------------+
| 1. Bootstrap: Install FiberScheduler, reset VirtualClock to t=0        |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| 2. Fabric Setup: Instantiate UartBus, SPIBus, CANBus, and BusRegistry   |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| 3. Spawn Peripherals: Launch sensor & physics fibers (Prio 5 & 0)      |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| 4. Spawn Firmware: Launch EC firmware fiber running setup() & loop()   |
+------------------------------------------------------------------------+
                                    |
                                    v
+------------------------------------------------------------------------+
| 5. Stepper Loop: Advance VirtualClock (batch headless or interactive)  |
+------------------------------------------------------------------------+
```

---

## 2. Modes of Operation

### 1. Headless Batch Mode (CI / Automated Testing)
- Time is stepped as fast as possible using `VirtualClock::instance().run_until(max_us)`.
- 10 seconds of simulated firing sequence executes in a few hundred milliseconds of real wall-clock time.
- Emits pass/fail status code and exits upon completion.

### 2. Interactive / Real-Time Paced Mode
- Synchronizes virtual time steps with host wall-clock time or user keypresses.
- Bus snoopers route live ASCII/HEX dumps to dedicated terminal windows or tmux panes.
- Allows live fault injection (e.g., simulating disconnected pressure transducers or jammed stepper valves).

---

## 3. Canonical Runner Structure (`main.cpp`)

```cpp
#include <iostream>
#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"
#include "bus/UartBus.h"
#include "bus/UartSnooper.h"
#include "hal_mock/Arduino.h"

using namespace toad::sim;

// Forward declare firmware functions
extern void setup();
extern void loop();

int main(int argc, char** argv) {
    std::cout << "=== Starting Toad EC SITL Runner ===" << std::endl;

    // 1. Install fiber scheduler
    install_fiber_scheduler();
    VirtualClock::instance().reset(0);

    // 2. Initialize buses and snoopers
    auto rs485_bus = std::make_shared<UartBus>(115200, "RS-485");
    auto snooper = std::make_shared<UartSnooper>("RS-485", SnoopFormat::FORMAT_HEX_DUMP, &std::cout);
    rs485_bus->add_observer(snooper);

    // 3. Launch simulated peripherals
    // (e.g., launch_fiber_with_priority(PRIO_SENSORS, ...))

    // 4. Launch EC firmware fiber
    auto fw_fiber = launch_fiber_with_priority(PRIO_FIRMWARE, []() {
        setup();
        while (true) {
            loop();
            yield(); // Yields control if loop() does not delay
        }
    });

    // 5. Run simulation for 5 simulated seconds (5,000,000 us)
    VirtualClock::instance().run_until(5000000);

    std::cout << "=== Simulation Completed Successfully ===" << std::endl;
    return 0;
}
```

---

## 4. `UartTerminalHarness` & Interactive Tool

Headers: [`UartTerminalHarness.h`](UartTerminalHarness.h) | Executable: `build/uart_terminal_harness`

The terminal harness connects to a `UartBus` and allows two-way interactive debugging:
- Typing input into the console injects bytes directly into the bus toward firmware.
- Firmware responses are intercepted and formatted by `UartSnooper` in real time.

### CLI Usage:
```bash
# Interactive mode in standard terminal (ASCII format)
./build/uart_terminal_harness --format ascii

# Hex-dump mode
./build/uart_terminal_harness --format hexdump

# Allocate a dedicated POSIX pseudo-terminal (/dev/pts/X)
./build/uart_terminal_harness --pty
# Then in another terminal, connect via:
#   screen /dev/pts/3
```

