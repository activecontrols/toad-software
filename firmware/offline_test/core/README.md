# Core Simulation APIs (`offline_test/core/`)

This directory houses the foundational simulation runtime for the Toad Engine Controller Software-in-the-Loop (SITL) platform: **discrete-event virtual time** and **cooperative priority fiber scheduling**.

---

## Architecture Overview

Embedded firmware relies heavily on deterministic timing, delays (`delay()`, `delayMicroseconds()`), and cooperative multitasking. To simulate multi-rate firmware, physical sensor responses, and actuator dynamics without race conditions or host CPU jitter, the SITL uses:

1. **`VirtualClock`**: A discrete-event clock operating in integer microseconds (`uint64_t`). Time advances in instant discrete jumps between scheduled deadlines.
2. **`FiberScheduler`**: A priority-based cooperative scheduler built on `boost::fibers`. Ready tasks with higher priorities always execute before lower-priority tasks.

```
                  +--------------------------------+
                  |         VirtualClock           |
                  |     current_micros = 1250 us   |
                  +---------------+----------------+
                                  |
   +------------------------------+------------------------------+
   | (Priority 10)                | (Priority 5)                 | (Priority 0)
   v                              v                              v
+-----------------------+      +-----------------------+      +-----------------------+
|  EC Firmware Fiber    |      |   Sensor Sim Fibers   |      |  Actuator & Physics   |
| (setup() / loop())    |      | (ADS131M02, MAX31856) |      | (MksServo57D physics) |
+-----------------------+      +-----------------------+      +-----------------------+
```

---

## 1. `VirtualClock` API Reference

Header: [`VirtualClock.h`](VirtualClock.h)

`VirtualClock` is a singleton that manages virtual time and sleeping fibers.

### Singleton Access
```cpp
VirtualClock& clock = VirtualClock::instance();
```

### Time Queries
```cpp
uint64_t now_us() const; // Current virtual time in microseconds
uint32_t now_ms() const; // Current virtual time in milliseconds
```

### Fiber Sleep Methods
These functions suspend the calling fiber until virtual time reaches the requested timestamp. They **do not** block the host thread.
```cpp
void sleep_for(uint64_t delta_us);   // Suspend fiber for delta_us microseconds
void sleep_until(uint64_t target_us); // Suspend fiber until virtual time reaches target_us
```
> [!NOTE]
> The Arduino mock functions `delay(ms)` and `delayMicroseconds(us)` automatically delegate to `VirtualClock::instance().sleep_for()`.

### Simulation Stepping & Execution
Used by test fixtures and the runner to advance virtual time:
```cpp
// Reset clock to an initial microsecond value (default 0)
void reset(uint64_t initial_us = 0);

// Check if any fibers are currently sleeping in the timer queue
bool has_pending_timers() const;

// Microsecond timestamp of the nearest scheduled deadline (0 if queue is empty)
uint64_t next_deadline_us() const;

// Advance time to the next scheduled deadline and resume expired fibers.
// Returns true if an event was stepped to, false if the queue is empty.
bool step_to_next_event();

// Advance virtual time up to target_us, stepping through all intervening deadlines.
void advance_time_to(uint64_t target_us);

// Run the simulation loop until all pending timers are completed or max_us is reached.
void run_until(uint64_t max_us);
```

---

## 2. `FiberScheduler` API Reference

Header: [`FiberScheduler.h`](FiberScheduler.h)

The fiber scheduler provides deterministic cooperative priority scheduling. A ready fiber with higher priority will always be scheduled before a fiber with lower priority.

### Standard Priority Levels
Defined in namespace `toad::sim`:
```cpp
namespace toad::sim {
    constexpr int PRIO_FIRMWARE         = 10; // Highest: EC firmware execution
    constexpr int PRIO_SENSORS          = 5;  // Medium: ADC / Thermocouple sampling
    constexpr int PRIO_ACTUATOR_PHYSICS = 0;  // Lowest: Physical plant & motor models
}
```

### Initialization
Before launching priority fibers, install the scheduler algorithm on the running thread (typically at the start of `main()`):
```cpp
toad::sim::install_fiber_scheduler();
```

### Spawning Priority Fibers
Use the helper template `launch_fiber_with_priority` to spawn a fiber with an assigned priority:
```cpp
template <typename Fn, typename... Args>
boost::fibers::fiber launch_fiber_with_priority(int priority, Fn&& fn, Args&&... args);
```

---

## 3. `BusRegistry` API Reference

Header: [`BusRegistry.h`](BusRegistry.h) | Implementation: [`BusRegistry.cpp`](BusRegistry.cpp)

`BusRegistry` is a singleton service that maps microcontroller hardware pins `(rx_pin, tx_pin)` and named strings to virtual communication buses (`UartBus`):

```cpp
// Register an existing bus instance for a pin pair
toad::sim::BusRegistry::instance().register_uart(rx_pin, tx_pin, my_uart_bus);

// Retrieve or lazily create a bus on demand
auto bus = toad::sim::BusRegistry::instance().get_or_create_uart(rx_pin, tx_pin);

// Register by symbolic name (e.g. "RS485_6", "HW_CommsSerial")
toad::sim::BusRegistry::instance().register_named_uart("RS485_6", my_uart_bus);
```

---

## 4. Usage Example

```cpp
#include <iostream>
#include <cassert>
#include "VirtualClock.h"
#include "FiberScheduler.h"
#include "BusRegistry.h"
#include "Arduino.h"

using namespace toad::sim;

int main() {
    // 1. Install scheduler on current thread
    install_fiber_scheduler();

    // 2. Reset virtual clock to t=0
    VirtualClock::instance().reset(0);

    // 3. Launch simulated sensor fiber (Priority 5)
    auto sensor_fiber = launch_fiber_with_priority(PRIO_SENSORS, []() {
        while (micros() < 10000) {
            delay(1); // Sample every 1 ms (1000 us)
            std::cout << "[Sensor] Sampled at " << micros() << " us" << std::endl;
        }
    });

    // 4. Launch simulated firmware fiber (Priority 10)
    auto fw_fiber = launch_fiber_with_priority(PRIO_FIRMWARE, []() {
        while (micros() < 10000) {
            delay(2); // Control loop runs every 2 ms (2000 us)
            std::cout << "[Firmware] Control step at " << micros() << " us" << std::endl;
        }
    });

    // 5. Step simulation until 10 ms (10,000 us) has elapsed
    VirtualClock::instance().run_until(10000);

    sensor_fiber.join();
    fw_fiber.join();

    std::cout << "Simulation finished at " << micros() << " us." << std::endl;
    return 0;
}
```

---

## 4. `SimulatedGPIO` API Reference

Header: [`SimulatedGPIO.h`](SimulatedGPIO.h) | Implementation: [`SimulatedGPIO.cpp`](SimulatedGPIO.cpp)

`SimulatedGPIO` is a thread-safe singleton managing the electrical and logical state of microcontoller GPIO pins.

### Features:
- **Pin State Storage**: Tracks `PinMode` (`INPUT`, `OUTPUT`, etc.), digital levels (`LOW`, `HIGH`), and analog readings.
- **Edge Listeners**: Allows peripherals, bus muxes, or test harnesses to react synchronously to pin transitions (`LOW -> HIGH` or `HIGH -> LOW`).
- **Arduino HAL Mock Integration**: Powers Arduino API calls (`pinMode()`, `digitalWrite()`, `digitalRead()`, `analogWrite()`, `analogRead()`).

### Usage Example:
```cpp
#include "core/SimulatedGPIO.h"
#include "hal_mock/pins_arduino.h"

// 1. Configure mode and initial state
pinMode(PA0, arduino::OUTPUT);
digitalWrite(PA0, arduino::LOW);

// 2. Attach an edge listener (e.g. for an RS-485 select line)
toad::sim::SimulatedGPIO::instance().add_listener(PA0,
    [](uint32_t pin, arduino::PinStatus new_status, arduino::PinStatus old_status) {
        if (new_status == arduino::HIGH) {
            std::cout << "Pin " << pin << " transitioned HIGH" << std::endl;
        }
    });

// 3. Writing to pin triggers listener callback
digitalWrite(PA0, arduino::HIGH);
```

---

## 5. Next Steps (Future Milestones)

- **`SPIBusRegistry`**: Mapping for SPI bus chip selects (`CS`).
- **`attachInterrupt` Dispatch**: Linking falling/rising GPIO edges directly to simulated ISR callbacks.


