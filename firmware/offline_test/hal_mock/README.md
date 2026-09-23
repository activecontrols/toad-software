# Hardware Abstraction Layer (HAL) Mocks (`offline_test/hal_mock/`)

This directory provides host-compatible drop-in replacements for embedded Arduino and Teensy hardware headers. It intercepts hardware calls from production firmware (`src/` and `lib/`) and redirects them to the SITL simulation engine without requiring source code modifications.

---

## Architecture & Design Rationale

### 1. Official `ArduinoCore-API` Integration
Rather than maintaining hand-written mock implementations of Arduino core classes (`Print`, `Stream`, `HardwareSerial`, `String`, etc.), the build system fetches the official [ArduinoCore-API](https://github.com/arduino/ArduinoCore-API) via CMake `FetchContent`.

Key build flags configured in `offline_test/CMakeLists.txt`:
- `-DHOST`: Disables embedded-specific declarations like `atexit` in `Common.h`, ensuring clean compilation against the host glibc.
- `-DEXTENDED_PIN_MODE`: Configures `pin_size_t` as `uint32_t` to support 32-bit pin mapping and extended peripheral configurations.

This gives the simulation platform authentic Arduino interfaces out-of-the-box (including `print()`, `println()`, `printf()`, string manipulation, and standard constants like `OUTPUT`, `INPUT`, `HIGH`, `LOW`).

---

## Header & API Reference

### 1. `Arduino.h` & `Arduino.cpp`
Header: [`Arduino.h`](Arduino.h) | Implementation: [`Arduino.cpp`](Arduino.cpp)

Pulls in `api/ArduinoAPI.h` and implements time and pin operations:

#### Time Management (Wired to `VirtualClock`)
- `unsigned long millis()`: Returns current virtual time in milliseconds.
- `unsigned long micros()`: Returns current virtual time in microseconds.
- `void delay(unsigned long ms)`: Suspends the calling fiber for `ms * 1000` virtual microseconds.
- `void delayMicroseconds(unsigned int us)`: Suspends the calling fiber for `us` virtual microseconds.
- `void yield()`: Calls `boost::this_fiber::yield()` to yield execution to other ready fibers.

#### GPIO Functions (Wired to `SimulatedGPIO`)
- `void pinMode(pin_size_t pin, PinMode mode)`: Updates pin mode in `toad::sim::SimulatedGPIO::instance()`.
- `void digitalWrite(pin_size_t pin, PinStatus status)`: Sets digital state and dispatches edge change callbacks to registered listeners (e.g. `RS485Mux`).
- `PinStatus digitalRead(pin_size_t pin)`: Queries digital state from `SimulatedGPIO`.
- `int analogRead(pin_size_t pin)`: Queries simulated analog level.
- `void analogWrite(pin_size_t pin, int value)`: Sets simulated analog level.

---

### 2. `pins_arduino.h`
Header: [`pins_arduino.h`](pins_arduino.h)

Provides board pin definitions matching the TOAD_H7 microcontroller layout (`PA0`..`PG15`, `NC`), as well as standard Arduino pin modes and state constants:
- Imports `api/Common.h` from ArduinoCore-API.
- Aliases `PinStatus` and `PinMode` enum constants (`LOW`, `HIGH`, `CHANGE`, `FALLING`, `RISING`, `INPUT`, `OUTPUT`, `INPUT_PULLUP`, `INPUT_PULLDOWN`, `OUTPUT_OPENDRAIN`) into `namespace arduino` for seamless scoped or unscoped usage.

---

### 3. `SPI.h` & `SPI.cpp`
Header: [`SPI.h`](SPI.h) | Implementation: [`SPI.cpp`](SPI.cpp)

Defines the concrete `SPIClass` implementing `arduino::HardwareSPI`:
- **Full Arduino Compatibility**: Implements `begin()`, `end()`, `beginTransaction(SPISettings)`, `endTransaction()`, `transfer(uint8_t)`, `transfer16(uint16_t)`, and `transfer(void*, size_t)`.
- **Copy-by-Value Semantics**: Stores a `std::shared_ptr<toad::sim::SPIBus>` backend, allowing `SPIClass` to be passed by value (as in `ADS131M02(SPIClass spi_bus, ...)`) while ensuring all instances share the same virtual bus fabric.
- **Pre-defined Instances**: Defines weak default instances for `PT_TC_SPI_1` and `PT_TC_SPI_3` matching `hardware_mapping/ec_pins.h`, allowing test binaries to link without `ec_main.cpp`.
- **Lazy Resolution**: Resolves backend bus automatically via `BusRegistry::instance().get_or_create_spi(mosi, miso, sck)`.

---

### 4. `HardwareSerial.h`
Header: [`HardwareSerial.h`](HardwareSerial.h)

Defines the `Uart` class, inheriting from `arduino::HardwareSerial`:

```cpp
class Uart : public arduino::HardwareSerial {
public:
    Uart(uint32_t rx, uint32_t tx, uint32_t de = 0);
    virtual ~Uart() = default;

    void begin(unsigned long baud) override;
    void begin(unsigned long baud, uint16_t config) override;
    void end() override;

    size_t write(uint8_t byte) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    int read() override;
    int available() override;
    int peek() override;
    void flush() override;

    operator bool() override { return true; }

    uint32_t rx_pin() const;
    uint32_t tx_pin() const;
    uint32_t de_pin() const;

    void attach_bus(std::shared_ptr<toad::sim::UartBus> bus);
    std::shared_ptr<toad::sim::UartBus> get_bus() const;
};

// Emulated USB Serial for Teensy / Arduino CommsSerial compatibility
class USBSerial : public Uart {
public:
    USBSerial() : Uart(0, 0, 0) {}
};

extern Uart Serial;
```

#### How `Uart` Connects to the Simulation
1. When firmware calls `Uart::begin(baud)`, it lazily resolves its backend `UartBus` via `BusRegistry::instance().get_or_create_uart(rx, tx, de)` and updates the baud rate.
2. When firmware writes data (`Uart::write()`), bytes are sent to `UartBus::write_from_firmware()`, simulating wire latency in `VirtualClock`.
3. When firmware checks `Uart::available()` or calls `Uart::read()` / `Uart::peek()`, bytes are fetched from `UartBus` without polling the OS.

---

## Include Precedence in CMake

To allow production firmware to `#include <Arduino.h>` or `#include <HardwareSerial.h>` seamlessly, `offline_test/CMakeLists.txt` sets include directories in order of precedence:

1. `offline_test/hal_mock` (matches mock headers first)
2. `offline_test/core`
3. `offline_test/bus`
4. Fetched `arduinocore-api/api`
5. `firmware/include`
6. `firmware/lib/*`
7. `firmware/src`

This ensures that the compiler picks up mock headers during SITL builds without requiring any `#ifdef SIMULATION` guards in production code.

---

### 5. `CAN.h` & `CAN.cpp`
Header: [`CAN.h`](CAN.h) | Implementation: [`CAN.cpp`](CAN.cpp)

Defines the `CAN` class implementing `arduino::HardwareCAN` from `ArduinoCore-API`:
- **HardwareCAN Interface**: Implements `begin(CanBitRate const can_bitrate)`, `end()`, `write(const arduino::CanMsg& msg)`, `available()`, and `read()`.
- **Hardware Abstraction Separation**: Embedded mock layer remains strictly decoupled from simulation internals. CAN frames use standard `arduino::CanMsg`, and simulation types like `toad::sim::CanFrame` reside strictly within `bus/CanFrame.h`.
- **Pre-defined Instances**: Defines global instances `CAN_TVC` and `CAN_FC`.
- **Lazy Bus Binding**: Automatically resolves backend `CANBus` instances via `BusRegistry`.

