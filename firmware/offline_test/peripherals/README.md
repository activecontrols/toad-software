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
- `start(priority)` / `stop()` / `join()`: Pattern 2 fiber lifecycle methods.

#### Dual-Mode Execution (Synchronous or Pattern 2 Fiber):
- **Synchronous Mode (Default)**: If `start()` is not called, packets are processed immediately on `on_receive_bytes()`.
- **Pattern 2 Fiber Mode**: Calling `start(PRIO_SENSORS)` creates an internal `boost::fibers::buffered_channel<std::vector<uint8_t>>` message queue and launches an independent background fiber. Firmware packets are pushed to the queue and processed asynchronously by the peripheral fiber.

#### Rapid Substitution with `FunctionalRS485Device`:
Team members can substitute any peripheral model in 3 lines, with optional fiber execution:
```cpp
auto custom_motor = std::make_shared<FunctionalRS485Device>("MyCustomMotor",
    [](const uint8_t* data, size_t len, IRS485Device& dev) {
        // Parse incoming firmware packet
        if (len >= 1 && data[0] == 0x01) {
            VirtualClock::instance().sleep_for(50); // Simulate processing latency
            dev.transmit_to_bus("MOTOR_OK\n");
        }
    });

// Option A: Run on independent background fiber with message queue (Pattern 2)
custom_motor->start(PRIO_ACTUATOR_PHYSICS);

// Plug directly into RS485Mux on its select pin
mux->register_device(PIN_TVC_PITCH_SEL, custom_motor);
```

---

### `ISPIDevice` & `FunctionalSPIDevice`
Header: [`ISPIDevice.h`](ISPIDevice.h)

An abstract interface enabling developers to model hardware ICs, sensors, and flash memory on an SPI bus:
- `name()`: Identifier for debug and transaction logging.
- `on_cs_asserted()`: Called when the device's Chip Select (`CS`) line is asserted (`LOW`).
- `on_cs_deasserted()`: Called when the device's Chip Select (`CS`) line is deasserted (`HIGH`).
- `transfer_byte(mosi)`: Full-duplex single byte transfer (returns MISO byte).
- `transfer_buffer(tx_buf, rx_buf, count)`: Full-duplex buffer transfer (defaults to sequentially invoking `transfer_byte`).
- `start(priority)` / `stop()` / `join()`: Fiber lifecycle methods for concurrent background execution.

#### Rapid Substitution with `FunctionalSPIDevice`:
Team members can substitute any SPI peripheral model in 3 lines using lambdas, with optional background fiber execution:
```cpp
auto adc_mock = std::make_shared<FunctionalSPIDevice>("ADS131M02_Mock",
    [](uint8_t mosi, FunctionalSPIDevice& dev) -> uint8_t {
        // Return synthetic ADC output byte on MISO
        return 0xA5;
    });

// Optional: run an independent background fiber worker
adc_mock->set_worker([](FunctionalSPIDevice& dev) {
    while (dev.is_running()) {
        VirtualClock::instance().sleep_for(1000);
        // update sensor state...
    }
});
adc_mock->start(PRIO_SENSORS);

// Plug directly into SPIBus on its CS pin
spi_bus->register_device(PIN_PT_BOARD_1_2_CS, adc_mock);
```

---

### `ADS131M02_Sim`
Header: [`ADS131M02_Sim.h`](ADS131M02_Sim.h) | Implementation: [`ADS131M02_Sim.cpp`](ADS131M02_Sim.cpp)

Behavioral model of the Texas Instruments ADS131M02 24-bit simultaneous-sampling dual-channel ADC used for Chamber and Manifold pressure sensing:
- **Concurrent Conversion Fiber**:
  - `start(priority)` launches an independent background fiber running at `PRIO_SENSORS = 5` (default 1000 us = 1 kHz conversion rate).
  - Generates 4-word (12-byte) SPI frames matching hardware datasheet (Status Word, Channel 0 24-bit reading, Channel 1 24-bit reading, and CCITT-CRC16).
  - Toggles `drdy_pin` via `SimulatedGPIO` on conversion complete to notify MCU interrupt routines.
- **Controls & Testing**:
  - `set_ch0_raw(counts)` / `set_ch1_raw(counts)`: Sets 24-bit signed ADC conversion counts (-8,388,608 to 8,388,607).
  - `set_sample_period_us(us)`: Configures conversion rate (default 1000 us).
  - `set_drdy_pin(pin)`: Configures data-ready interrupt pin.
  - `calculate_crc(data, len)`: Hardware-accurate CCITT-CRC16 generator.

---

### `AMT242AV_Sim`
Header: [`AMT242AV_Sim.h`](AMT242AV_Sim.h) | Implementation: [`AMT242AV_Sim.cpp`](AMT242AV_Sim.cpp)

Behavioral model of the Broadcom/CUI AMT242AV 12-bit modular absolute rotary shaft encoder over RS-485:
- **Pattern 2 Fiber Queue**:
  - `start(priority)` launches an independent background fiber (default `PRIO_SENSORS = 5`) consuming from an internal `boost::fibers::buffered_channel<std::vector<uint8_t>>`.
  - Simulates physical response latency (`set_response_delay_us(70)` defaults to 70 us, matching hardware datasheet).
  - Simulates controller reboot delay (`set_reset_delay_us(5000)` defaults to 5 ms).
  - `stop()` and `join()` for clean fiber teardown.
- **Command Handling**:
  - `ID` (e.g. `0x00`): Read Position. Transmits 2 bytes containing 12-bit angular position formatted across bits 2..13 and 2-bit odd parity checksum across bits 14..15.
  - `ID | 0x02`: Set zero position reference (zeroes output position).
  - `ID | 0x03`: Reset controller (reboots controller, delays by default 5 ms, preserves absolute position).
- **Test Controls**:
  - `set_position_fraction(0.5f)`: Sets shaft angle as normalized 0.0 - 1.0 fraction.
  - `set_raw_position(pos12)`: Sets explicit 12-bit raw tick value (0..4095).
  - `set_response_delay_us(us)`: Configures command processing latency (default 70 us).
  - `set_reset_delay_us(us)`: Configures reboot delay duration (default 5000 us = 5 ms).
  - `reset_count()`: Inspects number of controller resets performed.
  - `calculate_checksum(pos12)`: Computes hardware 2-bit odd parity.

---

### `ICANDevice` & `FunctionalCANDevice`
Header: [`ICANDevice.h`](ICANDevice.h)

An abstract interface enabling developers to model hardware nodes, actuators, and controllers on a multi-drop CAN bus:
- `name()`: Node identifier.
- `can_id()`: Configured CAN node identifier (e.g. `0x003`).
- `can_mask()`: Acceptance mask (default `0x7FF` for 11-bit standard frames).
- `enqueue_frame(frame)`: Delivers broadcast frame to the node's private queue.
- `start(priority)` / `stop()` / `join()`: Fiber lifecycle methods for concurrent background execution.

#### Rapid Substitution with `FunctionalCANDevice`:
```cpp
auto mock_node = std::make_shared<FunctionalCANDevice>("MockNode", 0x120,
    [](const CanFrame& frame, FunctionalCANDevice& dev) {
        // Handle incoming frame...
    });
mock_node->start(PRIO_ACTUATOR_PHYSICS);
can_bus->subscribe(mock_node);
```

---

### `MksServo57D_Sim`
Header: [`MksServo57D_Sim.h`](MksServo57D_Sim.h) | Implementation: [`MksServo57D_Sim.cpp`](MksServo57D_Sim.cpp)

Behavioral model of the Makerbase MKS SERVO42D / 57D closed-loop stepper motor controller:
- **Concurrent Physics Integration Fiber**:
  - `start(priority)` runs a background fiber on `PRIO_ACTUATOR_PHYSICS = 0` with a 1 ms (1000 µs) timestep.
  - Velocity ramping smoothed by `acceleration_`.
  - Continuous angular position integration (`current_angle_deg_`).
- **Hardware Protocols**:
  - Validates hardware checksum: `crc = can_id + sum(bytes)`. Rejects invalid checksum frames (`crc_error_count()`).
  - Decodes Speed & Acceleration commands (`0xF6`, signed speed, acceleration).
  - Handles status queries (`0x30`, `0x36`) and replies with telemetry frames (`0x31`, speed, angle, checksum).
- **Safe Teardown**:
  - Calls `VirtualClock::instance().wake_all()` in `stop()`, preventing deadlocks during fiber joins.

---

### `TVCActuator_Sim`
Header: [`TVCActuator_Sim.h`](TVCActuator_Sim.h) | Implementation: [`TVCActuator_Sim.cpp`](TVCActuator_Sim.cpp)

Behavioral model of the Thrust Vector Control (TVC) pitch/yaw linear actuator:
- **Concurrent Dynamics Fiber**:
  - `start(priority)` runs on `PRIO_ACTUATOR_PHYSICS = 0`.
  - Simulates linear stroke length (0.0 to 100.0 mm) moving towards setpoint at 25 mm/s.
- **Commands & Telemetry**:
  - Set Position (`0x20`, length in 0.1 mm units).
  - Status Query (`0x21`) and Status Reply (`0x22`).
- **Safe Teardown**:
  - Calls `VirtualClock::instance().wake_all()` in `stop()`.

---

## 3. Planned Peripherals

| Device | Type | Interface | Description |
|---|---|---|---|
| **`MAX31856_Sim`** | Sensor | SPI | Precision thermocouple amplifier with cold-junction compensation for exhaust/plumbing temperatures. |
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

