# Simulated Peripherals (`offline_test/peripherals/`)

This directory houses behavioral simulation models for hardware ICs, sensors, actuators, and physical plants used by the Engine Controller.

---

## 1. Concurrency & Simulation Model

Each simulated hardware device operates as an **independent cooperative fiber** with its own execution context:

- **Sensors (Priority `toad::sim::PRIO_SENSORS = 5`)**:
  Simulate ADC conversions, thermocouple integration, encoder readings, and interrupt assertions (`DRDY`).
- **Actuators & Physics (Priority `toad::sim::PRIO_ACTUATOR_PHYSICS = 0`)**:
  Simulate mechanical movements, motor velocities, and physical pressure/flow dynamics.

Peripherals never use host threads or wall-clock timers. All delays and conversion timing use `VirtualClock::instance().sleep_for()` or `delayMicroseconds()`, ensuring 100% deterministic execution.

```
                           +------------------------+
                           |  Simulated SPI / UART  |
                           +-----------+------------+
                                       |
                   +-------------------+-------------------+
                   | (CS Pin 10)                           | (CS Pin 9)
                   v                                       v
        +--------------------+                  +--------------------+
        |   ADS131M02_Sim    |                  |    MAX31856_Sim    |
        |  (24-bit ADC Sim)  |                  | (Thermocouple Sim) |
        |   Priority: 5      |                  |    Priority: 5     |
        +--------------------+                  +--------------------+
```

---

## 2. Implemented & Available Peripherals

### `IRS485Device` & `FunctionalRS485Device`
Header: [`IRS485Device.h`](IRS485Device.h)

An abstract interface enabling developers to plug custom models (motors, sensors, actuators) into `RS485Mux`:
- `name()`: Identifier for debug/snooping.
- `on_receive_bytes(buffer, size)`: Called when firmware transmits to the device while its `SEL` pin is `HIGH`.
- `transmit_to_bus(buffer, size)`: Convenience method for peripheral models to reply onto the bus.

#### Rapid Substitution with `FunctionalRS485Device`:
Team members can substitute any peripheral model (e.g. custom motor, actuator, valve) in 3 lines without subclassing:
```cpp
auto custom_motor = std::make_shared<FunctionalRS485Device>("MyCustomMotor",
    [](const uint8_t* data, size_t len, IRS485Device& dev) {
        // Parse incoming firmware packet
        if (len >= 1 && data[0] == 0x01) {
            // Reply back to firmware
            dev.transmit_to_bus("MOTOR_OK\n");
        }
    });

// Plug directly into RS485Mux on its select pin
mux->register_device(PIN_TVC_PITCH_SEL, custom_motor);
```

---

### `AMT242AV_Sim`
Header: [`AMT242AV_Sim.h`](AMT242AV_Sim.h) | Implementation: [`AMT242AV_Sim.cpp`](AMT242AV_Sim.cpp)

Behavioral model of the Broadcom/CUI AMT242AV 12-bit modular absolute rotary shaft encoder over RS-485:
- **Command Handling**:
  - `ID` (e.g. `0x00`): Read Position. Transmits 2 bytes containing 12-bit angular position formatted across bits 2..13 and 2-bit odd parity checksum across bits 14..15.
  - `ID | 0x02`: Set zero position reference (zeroes output position).
  - `ID | 0x03`: Reset controller (reboots controller, delays by default 5 ms, preserves absolute position).
- **Test Controls**:
  - `set_position_fraction(0.5f)`: Sets shaft angle as normalized 0.0 - 1.0 fraction.
  - `set_raw_position(pos12)`: Sets explicit 12-bit raw tick value (0..4095).
  - `set_reset_delay_us(us)`: Configures reboot delay duration (default 5000 us = 5 ms).
  - `reset_count()`: Inspects number of controller resets performed.
  - `calculate_checksum(pos12)`: Computes hardware 2-bit odd parity.

---

## 3. Planned Peripherals

| Device | Type | Interface | Description |
|---|---|---|---|
| **`ADS131M02_Sim`** | Sensor | SPI | 24-bit dual-channel simultaneous-sampling ADC for Chamber & Manifold Pressure. Simulates data-ready (`DRDY`) interrupts. |
| **`MAX31856_Sim`** | Sensor | SPI | Precision thermocouple amplifier with cold-junction compensation for exhaust/plumbing temperatures. |
| **`MksServo57D_Sim`** | Actuator | RS-485 (UART) | Closed-loop stepper motor controller modeling position commands and valve angle physics. |
| **`Solenoid_Sim`** | Actuator | GPIO | Digital output solenoid valves for igniter, purge, and pressurization valves. |

---

## 4. Peripheral Developer Guide: Creating a New Simulated Device

To create a new simulated peripheral:
1. Inherit from the relevant bus device interface (e.g., `ISPIDevice` or connect to `UartBus`).
2. Provide a `start()` method that launches its background fiber with appropriate priority.
3. Update simulated registers or internal states on clock ticks or when commanded over the bus.

### Example Template: Simulated SPI Sensor
```cpp
#pragma once
#include <cstdint>
#include <memory>
#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"

class SampleSensor_Sim {
public:
    SampleSensor_Sim(uint32_t cs_pin) : cs_pin_(cs_pin) {}

    void start() {
        fiber_ = toad::sim::launch_fiber_with_priority(
            toad::sim::PRIO_SENSORS,
            [this]() { this->run(); }
        );
    }

    void join() {
        if (fiber_.joinable()) fiber_.join();
    }

    // Called when the SPI bus sends a byte to this device while CS is active
    uint8_t transfer_byte(uint8_t mosi) {
        // Return simulated register data
        return current_reading_++;
    }

private:
    void run() {
        while (running_) {
            // Model 1000 us (1 ms) ADC conversion time
            VirtualClock::instance().sleep_for(1000);
            
            // Update sensor value or assert DRDY interrupt pin
            current_reading_ = compute_simulated_physical_value();
        }
    }

    uint8_t compute_simulated_physical_value() {
        return 42; // Replace with synthetic wave or physical model
    }

    uint32_t cs_pin_;
    bool running_{true};
    uint8_t current_reading_{0};
    boost::fibers::fiber fiber_;
};
```

