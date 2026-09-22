# Bus Emulation & Transaction Snooping (`offline_test/bus/`)

This directory houses the virtual bus fabrics (`UartBus`, `SPIBus`, `RS485Mux`, `CANBus`) and transaction inspection utilities (`ITransactionObservable`, `UartSnooper`).

---

## 1. 3-Layer Bus Architecture

The simulation strictly decouples firmware code from the underlying physical transmission medium and inspection tools:

```
+--------------------------------------------------------------------------+
| Layer 1: Firmware Controller Interface (hal_mock/)                       |
|   - Uart (HardwareSerial.h)                                              |
|   - SPIClass (SPI.h)                                                     |
|   - FlexCAN_T4 (MockCAN.h)                                               |
+--------------------------------------------------------------------------+
                                    |
                                    v
+--------------------------------------------------------------------------+
| Layer 2: Simulated Bus Fabric (bus/)                                     |
|   - UartBus / RS485Mux                                                   |
|   - SPIBus                                                               |
|   - CANBus                                                               |
|   * Simulates propagation delay, baud rate transmission timing, and FIFO *
+--------------------------------------------------------------------------+
          |                                                    |
          v                                                    v
+----------------------------------------+   +-----------------------------+
| Layer 3: Inspection Snooper (bus/)     |   | Simulated Hardware Fiber    |
|   - ITransactionObservable             |   | (peripherals/)              |
|   - UartSnooper / SPISnooper           |   | - Stepper controller        |
|   * Formats ASCII / HEX / HEX DUMP     |   | - Pressure transducer ADC   |
|   * Dispatches to dedicated terminals  |   | - Thermocouple amplifier    |
+----------------------------------------+   +-----------------------------+
```

---

## 2. Components & APIs

### `ITransactionObservable`
Header: [`ITransactionObservable.h`](ITransactionObservable.h)

An abstract interface implemented by all simulation buses to allow non-intrusive transaction sniffing:

```cpp
namespace toad::sim {

enum class BusDirection {
    FW_TO_BUS, // Transmitted by firmware into the bus
    BUS_TO_FW  // Transmitted by peripheral/harness into firmware
};

struct UartTransaction {
    uint64_t timestamp_us{0};
    BusDirection direction{BusDirection::FW_TO_BUS};
    std::vector<uint8_t> data;
};

class IUartObserver {
public:
    virtual ~IUartObserver() = default;
    virtual void on_uart_transaction(const UartTransaction& tx) = 0;
};

class IUartObservable {
public:
    virtual ~IUartObservable() = default;
    virtual void add_observer(std::shared_ptr<IUartObserver> observer) = 0;
    virtual void remove_observer(std::shared_ptr<IUartObserver> observer) = 0;
};

} // namespace toad::sim
```

---

### `UartBus`
Header: [`UartBus.h`](UartBus.h) | Implementation: [`UartBus.cpp`](UartBus.cpp)

Simulates point-to-point and multi-drop serial channels between firmware UART ports and peripheral models:

- **Baud Rate Timing**: Models byte transmission duration based on configured baud rate and advances virtual time accordingly in `VirtualClock`.
- **Non-Destructive Simulation Peeking**:
  - `peek_fw_tx()`: Inspects the next byte waiting in the firmware TX queue without removing it.
  - `peek_all_fw_tx()`: Returns a snapshot of all queued bytes without popping them.
  - `get_transaction_history()`: Retrieves logged transaction records with timestamps.
- **Queues**: Thread/fiber-safe dual-channel FIFO buffers for firmware transmission (`write_from_firmware`, `read_for_firmware`) and external peripheral interaction (`write_to_firmware`, `read_from_firmware`).

---

### `UartSnooper`
Header: [`UartSnooper.h`](UartSnooper.h) | Implementation: [`UartSnooper.cpp`](UartSnooper.cpp)

Observes raw byte streams crossing a `UartBus` and renders formatted output in real time.

#### Formatting Modes (Configured Per-Bus)
- `SnoopFormat::FORMAT_ASCII`: Printable text with line timestamps; non-printable characters escaped (`\r`, `\n`, `\xNN`).
- `SnoopFormat::FORMAT_HEX`: Space-delimited byte hex (`[1250 us] [UART FW -> BUS] (4 bytes): 0x70 0x6F 0x6E 0x67`).
- `SnoopFormat::FORMAT_HEX_DUMP`: Classic side-by-side offset, 16-byte hex representation, and ASCII representation.

#### Output Routing
Snoopers can direct formatted output to:
1. **Standard stream**: `set_output_stream(&std::cout)`
2. **Custom callback**: `set_custom_sink([](const std::string& line) { ... })`
3. **File descriptor / POSIX PTY**: `set_fd_sink(pty_master_fd)`

---

## 3. Usage Pattern

```cpp
#include "bus/UartBus.h"
#include "bus/UartSnooper.h"
#include "core/BusRegistry.h"

// 1. Create simulated bus fabric
auto rs485_bus = std::make_shared<toad::sim::UartBus>(/*baud=*/115200, "RS485_6");

// 2. Attach a snooper configured for hex dump output
auto snooper = std::make_shared<toad::sim::UartSnooper>(
    "RS485_6",
    toad::sim::SnoopFormat::FORMAT_HEX_DUMP,
    &std::cout
);
rs485_bus->add_observer(snooper);

// 3. Register bus for firmware pin pair
toad::sim::BusRegistry::instance().register_uart(/*rx_pin=*/0, /*tx_pin=*/1, rs485_bus);
```

---

### `RS485Mux`
Header: [`RS485Mux.h`](RS485Mux.h) | Implementation: [`RS485Mux.cpp`](RS485Mux.cpp)

Extends `UartBus` to simulate half-duplex multi-drop RS-485 busses where peripherals are switched onto the physical bus via dedicated Select (`SEL`) GPIO lines (such as `PIN_ENC_OX_SEL` and `PIN_TVC_PITCH_SEL` on `RS485_6`).

#### Key Capabilities:
- **`SEL` Pin Demultiplexing**: Peripherals register with their respective `sel_pin` via `register_device(sel_pin, device)`.
- **Automatic GPIO Listener Wiring**: The mux hooks into `toad::sim::SimulatedGPIO::instance()` to monitor digital pin changes in real time.
- **Hardware Bus Contention Detection**: If firmware or buggy code asserts multiple `SEL` lines `HIGH` simultaneously, the mux detects electrical collision (`has_bus_contention() == true`), increments `contention_count()`, and drops corrupt bus traffic.
- **Pluggable Peripheral Forwarding**: When exactly one `SEL` line is `HIGH`, incoming firmware UART traffic (`write_from_firmware`) is immediately routed to that device's `on_receive_bytes()`. Device replies via `dev.transmit_to_bus()` are validated and routed back to firmware receive FIFO (`read_for_firmware()`).

```cpp
#include "bus/RS485Mux.h"
#include "peripherals/AMT242AV_Sim.h"
#include "peripherals/IRS485Device.h"
#include "hardware_mapping/ec_pins.h"

// 1. Create RS-485 Mux bus fabric
auto mux = std::make_shared<toad::sim::RS485Mux>(115200, "RS485_6", PIN_RS485_6_DE);

// 2. Attach simulated encoder to PIN_ENC_OX_SEL
auto encoder = std::make_shared<toad::sim::AMT242AV_Sim>(0x00, "OxShaftEncoder");
mux->register_device(PIN_ENC_OX_SEL, encoder);

// 3. Attach custom motor/actuator mock to PIN_TVC_PITCH_SEL
auto motor = std::make_shared<toad::sim::FunctionalRS485Device>("PitchMotor",
    [](const uint8_t* data, size_t len, toad::sim::IRS485Device& dev) {
        dev.transmit_to_bus("ACK\n");
    });
mux->register_device(PIN_TVC_PITCH_SEL, motor);

// 4. Register to BusRegistry for firmware Uart(PIN_RS485_6_RX, PIN_RS485_6_TX)
toad::sim::BusRegistry::instance().register_uart(PIN_RS485_6_RX, PIN_RS485_6_TX, mux);
```

---

## 4. Other Bus Fabrics (Future Milestones)

- **`SPIBus`**: Simulates synchronous master-slave clocking, chip select (`CS`) line decoding, and full-duplex transfers.
- **`CANBus`**: Simulates CAN 2.0B / CAN FD arbitrated multi-drop buses with message ID filtering and priority resolution.


