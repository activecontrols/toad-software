# Toad Firmware: Offline Test Platform Architecture & Design Plan

This document outlines the architecture, design decisions, and implementation plan for the **Engine Controller (EC) Offline Simulation & Test Platform**. 

The platform is housed within `firmware/offline_test/` and built using **CMake** with **vcpkg** (in manifest mode via `vcpkg.json`) to manage external dependencies (`boost-fiber`, `boost-context`). It operates as a self-contained Software-in-the-Loop (SITL) environment that compiles and runs production firmware files while cleanly mocking hardware-dependent interfaces (SPI, UART/RS-485, CAN, GPIO, and Clock).

> [!NOTE]
> **Scope & Future Extensibility**: While this initial phase focuses specifically on **Virtual Time-Stepping** and **Bus Emulation** for the **Engine Controller**, the current SITL architecture intentionally factors in the needs of the future testing environment to allow easy expansion. Key architectural decisions—such as separating bus fabrics from firmware wrappers, vtable-based transaction observability (`ITransactionObservable`), and isolating in-process `VirtualCAN` from cross-process `SystemCAN` sockets—are deliberately structured so that future capabilities (including multi-process Flight Controller simulation, inter-process bridges, and Dear ImGui visual dashboards) can be layered cleanly onto this foundation without architectural rework.

---

## 1. Architectural Principles

1. **Zero Modifications to Production Firmware**:
   The simulator must link [`src/ec_main.cpp`](../src/ec_main.cpp) and existing libraries in [`lib/`](../lib/) without modifying source code. Hardware interfaces are intercepted via CMake include-path precedence and drop-in mock classes.
2. **Cooperative Virtual Time (No Wall-Clock Coupling)**:
   Execution speed is decoupled from host PC clock speed. Time advances deterministically in discrete jumps to the next scheduled event, guaranteeing 100% reproducible test runs with zero timer jitter.
3. **Hardware Emulation via Controller-Bus Separation**:
   Firmware interacts with mock Controller classes (`SPIClass`, `Uart`, `CAN`), which delegate physical routing to simulated Bus/Fabric components (`SPIBus`, `UartBus` / `RS485Mux`, `CANBus`).
4. **Single-Process Concurrency with Fibers**:
   Using `boost::fiber` rather than OS threads (`std::thread` / `boost::thread`) gives us complete, deterministic control over time stepping and scheduling that is impossible with normal preemptive threads. With OS threads, the host kernel controls when threads yield and resume, making virtual time stepping prone to race conditions and unpredictable preemption. With fibers, execution is strictly cooperative and managed by our custom scheduler—allowing us to step time, pause, and jump between deadlines with total precision.
5. **Dependency Management with vcpkg**:
   Host dependencies (`boost-fiber`, `boost-context`) are managed via `vcpkg` in manifest mode (`vcpkg.json`). This ensures reproducible, isolated Boost builds without relying on system-wide package manager version quirks.

---

## 2. Concurrency & Virtual Time-Stepping

### 2.1 Why `boost::fiber`? (Control Over Time & FreeRTOS Familiarity)

The primary motivation for choosing `boost::fiber` over standard OS threads is **direct, fine-grained control over the simulation clock and scheduling**:

- **The Problem with Normal OS Threads**:
  Normal OS threads are preemptive and scheduled asynchronously by the host operating system kernel. The host kernel decides when a thread gets paused or interrupted. If you attempt to run virtual time with normal threads, you cannot cleanly freeze time, detect when all threads are waiting without racy polling, or guarantee exact step intervals. A host CPU workload spike or context-switch delay directly skews simulated timings.
- **Why Fibers Provide Total Time Control**:
  Fibers are cooperative, user-space execution contexts. A fiber only yields when it explicitly calls `delay()`, waits on a channel, or yields. Because our simulation code controls the fiber scheduler, we have 100% visibility into the state of every simulated task: we know exactly which tasks are waiting, what their deadlines are, and when to advance the clock. This allows us to single-step microsecond by microsecond, pause indefinitely, or fast-forward without host timer jitter.
- **FreeRTOS Mental Model**:
  Fibers match the FreeRTOS paradigm familiar to embedded engineers:
  - **Independent Task Contexts**: Each fiber has its own private stack and execution pointer.
  - **Single-Core Sequential Execution**: Only one fiber executes at any given microsecond. There is no multi-core memory reordering, race condition, or cache invalidation.
  - **Lock-Free Bus Communications**: Inter-fiber communication uses channels (`boost::fibers::buffered_channel<T>`), directly analogous to FreeRTOS queues (`xQueueSend` / `xQueueReceive`).

```
                    +------------------------------------+
                    |       SIMULATION PROCESS           |
                    |                                    |
                    |  +------------------------------+  |
                    |  |       Virtual Clock          |  |
                    |  |   current_micros = 1250 us   |  |
                    |  +--------------+---------------+  |
                    |                 |                  |
   +----------------+-----------------+---------------+--+----------------+
   | (Priority 10)  | (Priority 5)    | (Priority 5)  | (Priority 0)      |
   v                v                 v               v                   v
+-------------+  +-------------+  +-------------+  +-----------------+  +-----------------+
| EC Firmware |  | PT ADC Sim  |  | TC Amp Sim  |  | OX Encoder Sim  |  | Stepper Physics |
| setup/loop  |  | (ADS131M02) |  | (MAX31856)  |  | (AMT242AV)      |  | (MksServo57D)   |
+-------------+  +-------------+  +-------------+  +-----------------+  +-----------------+
```

### 2.2 Priority Scheduling

Boost.Fiber provides priority scheduling via `boost::fibers::algo::priority_props`:
- **Priority Hierarchy**:
  1. `PRIO_FIRMWARE` (High, e.g., 10): Primary EC control loop and command handling.
  2. `PRIO_SENSORS` (Medium, e.g., 5): ADC and thermocouple digital updates.
  3. `PRIO_ACTUATOR_PHYSICS` (Low, e.g., 0): Background stepper motor movement and valve dynamics.
- **Guarantee**: Lower-priority tasks (like stepper motor physics) are **blocked** from running as long as higher-priority tasks (firmware, command routing) are runnable. They execute only when all higher-priority tasks are waiting on timers or bus I/O.
- **Simultaneous Wakeups**: If Firmware and a Stepper wake up at the exact same microsecond (`T = 5000 us`), the priority scheduler guarantees Firmware runs first.

### 2.3 Microsecond Virtual Clock (`VirtualClock`)

In the real world, OS timers have millisecond granularity and context-switch jitter. In this test platform, virtual time is an integer counter (`uint64_t current_micros`).

#### Discrete-Event Jumping (Avoid Fixed Polling)
A naive 10 us tick would require 300,000 iterations to advance through a 3-second boot delay (`delay(3000)` in [`src/ec_main.cpp`](../src/ec_main.cpp#L42)). Instead, the clock uses **deadline-driven jumps**:

1. A fiber calls `delay(x)` or `delayMicroseconds(us)`.
2. The fiber calculates its deadline: `T_target = T_now + delta_t` and registers this in a priority queue.
3. The fiber yields/blocks.
4. When all fibers are blocked waiting on timers, the simulation engine identifies the earliest deadline:
   `T_next = min(T_target1, T_target2, ...)`
5. The clock **jumps immediately** to `T_next`, waking up the appropriate fibers.

#### Pacing the Firmware `loop()`
Arduino's `loop()` in [`src/ec_main.cpp`](../src/ec_main.cpp#L70-L74) contains no internal delays; it polls `CommsSerial.available()` continuously. 
To prevent the firmware fiber from starving the rest of the simulation at `T = 0`, the test harness inserts an implicit pacing tick at the end of each `loop()` iteration (e.g., `delayMicroseconds(10)`), modeling the execution time of an idle loop on an STM32H7.

---

## 3. Bus Emulation Architecture

The simulation decomposes each bus into a **Controller** (which satisfies the firmware API) and a **Bus Fabric** (which models routing, contention, and physical pin dynamics).

```
   Layer 1: Firmware API Layer
   ┌─────────────────────────────────────────────────────────────┐
   │       SPIClass               Uart                  CAN      │
   └──────────┬─────────────────────┬────────────────────┬───────┘
              │ forwards            │ forwards           │ forwards
              ▼                     ▼                    ▼
   Layer 2: Simulation Fabric Layer (Implements ITransactionObservable)
   ┌──────────────────────┐ ┌──────────────────┐ ┌───────────────┐
   │        SPIBus        │ │     UartBus      │ │    CANBus     │
   │   (Routes by CS)     │ │ (Links endpoints │ │  (Broadcasts  │
   │                      │ │  & RS485Mux)     │ │   to queues)  │
   └──────────┬───────────┘ └───────┬──────────┘ └───────┬───────┘
              │ queues              │ queues             │ queues
              ▼                     ▼                    ▼
   Layer 3: Simulated Peripheral Layer
   ┌──────────────────────┐ ┌──────────────────┐ ┌───────────────┐
   │ ADS131M02 / MAX31856 │ │ AMT242AV Encoder │ │ MksServo57D / │
   │  Pressure & Temp     │ │   Throttle Valve │ │ TVC Actuators │
   └──────────────────────┘ └──────────────────┘ └───────────────┘
```

---

### 3.1 Hardware-to-Simulation Pin Mapping & Lazy Resolution

#### Context in Toad
In [`src/ec_main.cpp`](../src/ec_main.cpp#L14-L23), hardware buses are declared as global objects instantiated with pin definitions from [`lib/hardware_mapping/ec_pins.h`](../lib/hardware_mapping/ec_pins.h):

```cpp
SPIClass PT_TC_SPI_1(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
SPIClass PT_TC_SPI_3(PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK);

Uart RS485_6(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE);
Uart RS485_2(PIN_RS485_2_RX, PIN_RS485_2_TX, PIN_RS485_2_DE);
```

#### The Wrapper Pattern
The mock `SPIClass` and `Uart` classes act as lightweight wrappers. In their constructors, they record the configured pin numbers (`mosi`, `miso`, `sck` or `rx`, `tx`, `de`). When transactions occur, they query a central `BusRegistry` to forward all operations to the appropriate backend simulation bus (`SPIBus`, `RS485Mux`, or `UartBus`).

#### Avoiding the Static Initialization Order Fiasco (Lazy Binding)
Because `PT_TC_SPI_1` and `RS485_6` are global variables in `ec_main.cpp`, their constructors execute **before `main()` starts**. In C++, the order in which static global variables are initialized across different translation units is undefined. 

If the `SPIClass` or `Uart` constructor immediately queried a static registry map, that map might not have been constructed yet.

To prevent this issue cleanly, the wrappers use **Lazy Binding**:
1. The constructors only store the primitive pin integers and initialize their backend pointer (`backend_bus_ = nullptr`).
2. Resolution occurs lazily the first time `begin()` or a communication method is invoked:
   ```cpp
   class SPIClass {
   public:
       SPIClass(uint32_t mosi, uint32_t miso, uint32_t sck)
           : mosi_(mosi), miso_(miso), sck_(sck), backend_bus_(nullptr) {}

       void begin() {
           resolve_bus();
           backend_bus_->begin();
       }

       uint8_t transfer(uint8_t data) {
           resolve_bus();
           return backend_bus_->transfer(data);
       }

   private:
       void resolve_bus() {
           if (backend_bus_ == nullptr) {
               backend_bus_ = BusRegistry::get_spi_bus(mosi_, miso_, sck_);
           }
       }

       uint32_t mosi_, miso_, sck_;
       SPIBus* backend_bus_;
   };

   class Uart : public arduino::HardwareSerial {
   public:
       Uart(uint32_t rx, uint32_t tx, uint32_t de = 0)
           : rx_(rx), tx_(tx), de_(de), backend_bus_(nullptr) {}

       void begin(unsigned long baud) override {
           resolve_bus();
           backend_bus_->begin(baud);
       }

       size_t write(uint8_t byte) override {
           resolve_bus();
           return backend_bus_->write(byte);
       }

       int read() override {
           resolve_bus();
           return backend_bus_->read();
       }

       int available() override {
           resolve_bus();
           return backend_bus_->available();
       }

       int peek() override {
           resolve_bus();
           return backend_bus_->peek();
       }

       void flush() override {
           resolve_bus();
           backend_bus_->flush();
       }

       operator bool() override {
           return true;
       }

   private:
       void resolve_bus() {
           if (backend_bus_ == nullptr) {
               backend_bus_ = BusRegistry::get_uart_bus(rx_, tx_, de_);
           }
       }

       uint32_t rx_, tx_, de_;
       UartBus* backend_bus_;
   };
   ```
3. Because `begin()` is called inside `setup()` (which runs after `main()` begins), all simulation registries, bus fabrics, and simulated peripherals are guaranteed to be fully constructed.

#### Fail-Fast Pin Error Detection
The `BusRegistry` matches pins against known board definitions from `ec_pins.h`:
- **SPI Validation**:
  - `(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK)` -> routes to `sim_spi_bus_1`.
  - `(PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK)` -> routes to `sim_spi_bus_3`.
  - Unknown or swapped pins abort: `"Simulation Error: Unmapped SPI pins! MOSI=PD7, MISO=PB4, SCK=PG12"`.
- **UART Validation**:
  - `(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE)` -> routes to `sim_rs485_bus_6` (an `RS485Mux` instance of `UartBus`).
  - `(PIN_RS485_2_RX, PIN_RS485_2_TX, PIN_RS485_2_DE)` -> routes to `sim_rs485_bus_2` (an `RS485Mux` instance of `UartBus`).
  - Unknown or swapped pins abort: `"Simulation Error: Unmapped UART pins! RX=PF9, TX=PF7, DE=PG12"`.

---

### 3.2 SPI Emulation (`SPIClass` & `SPIBus`)

#### Real Hardware Context
The EC uses two SPI buses:
- `PT_TC_SPI_1` and `PT_TC_SPI_3` (defined in [`src/ec_main.cpp`](../src/ec_main.cpp#L21-L22)).
- Attached devices: ADS131M02 ADCs for pressure sensors ([`lib/pressure_sensors/PressureSensors.cpp`](../lib/pressure_sensors/PressureSensors.cpp)) and MAX31856 amplifiers for thermocouples ([`lib/temperature_sensors/TemperatureSensors.cpp`](../lib/temperature_sensors/TemperatureSensors.cpp)).

#### Design
- **`ISpiDevice` Interface**:
  ```cpp
  class ISpiDevice {
  public:
      virtual ~ISpiDevice() = default;
      virtual void on_cs_assert() {}
      virtual void on_cs_deassert() {}
      virtual uint8_t transfer(uint8_t tx_byte) = 0;
  };
  ```
- **Queue-Based Master-Slave Routing (`SPIBus`)**:
  - `SPIClass::transfer(tx)` directly sends a transaction request through a queue to the `SPIBus`.
  - The `SPIBus` inspects `SimulatedGPIO` to verify which slave device has its Chip Select (`CS`) pin asserted `LOW`.
  - `SPIBus` forwards the data byte through a queue to the selected device, and the slave device pushes its response byte (MISO) back through a queue to the `SPIBus`, which delivers it back to `SPIClass::transfer()`.
  - If no device CS is `LOW`, the bus immediately returns `0xFF` (simulating bus pull-up behavior).
- **Concurrency Separation**:
  - The simulated sensor model (e.g. `ADS131M02_Sim`) runs an independent fiber updating internal ADC registers based on simulated pressure curves.
  - When the transfer queue delivers a request to the device, it reads its current register value and replies through the return queue.

---

### 3.3 UART & RS-485 Emulation (`Uart` & `UartBus`)

#### Real Hardware Context
The EC uses two hardware UARTs for RS-485 ([`lib/hardware_mapping/ec_pins.h`](../lib/hardware_mapping/ec_pins.h#L16-L40)):
- `RS485_6` (`PIN_RS485_6_DE` on `PG12`):
  - `PIN_TVC_PITCH_SEL` (`PF8`)
  - `PIN_ENC_OX_SEL` (`PF10`)
- `RS485_2` (`PIN_RS485_2_DE` on `PD4`):
  - `PIN_TVC_YAW_SEL` (`PD8`)
  - `PIN_ENC_FU_SEL` (`PD10`)

The AMT242AV absolute encoder ([`lib/throttle_valves/AMT242AV.cpp`](../lib/throttle_valves/AMT242AV.cpp)) drives its `SEL` pin `HIGH`/`LOW` to address the device across the shared transceiver. Direct serial links (such as `HW_CommsSerial`) are used for telemetry and ground communication.

#### The 3-Layer UART Architecture
Following the same design pattern as CAN and SPI, UART emulation is organized into three distinct layers:

```
 ┌──────────────────────────────────────────────────────────────┐
 │ Layer 1: Firmware API (hal_mock/HardwareSerial.h)            │
 │   - Class Uart implementing arduino::HardwareSerial          │
 │   - What ec_main.cpp and drivers call (write, read, etc.)    │
 └──────────────────────────────┬───────────────────────────────┘
                                │ forwards stream calls
                                ▼
 ┌──────────────────────────────────────────────────────────────┐
 │ Layer 2: Simulation Fabric (bus/UartBus.h & RS485Mux.h)      │
 │   - Class UartBus (sim-side only)                            │
 │   - Handles bidirectional stream link between UART endpoints │
 │   - Logic for simulation side to peek at transactions        │
 │   - Implements ITransactionObservable for future UI          │
 │   - RS485Mux specializes UartBus (routes by DE/SEL pins)     │
 └──────────────┬───────────────────────────────┬───────────────┘
                │ queues                        │ queues
                ▼                               ▼
 ┌──────────────────────────────┐┌──────────────────────────────┐
 │ Layer 3: Peripheral Model A  ││ Layer 3: Peripheral Model B  │
 │ (AMT242AV_Sim - OX Encoder)  ││ (AMT242AV_Sim - FU Encoder)  │
 │   - Blocks on RX queue       ││   - Blocks on RX queue       │
 │   - Responds via TX queue    ││   - Responds via TX queue    │
 └──────────────────────────────┘└──────────────────────────────┘
```

1. **Layer 1: Firmware API (`hal_mock/HardwareSerial.h`)**:
   - `class Uart : public arduino::HardwareSerial`. Inheriting from `arduino::HardwareSerial` (rather than just `Stream`) provides 1:1 parity with the STM32duino core.
   - **Zero Re-implementation Boilerplate**: Instead of manually mocking or re-implementing `Print`, `Stream`, or `HardwareSerial`, CMake directly links the official, open-source `ArduinoCore-API`. `HardwareSerial.h` simply includes `#include "api/HardwareSerial.h"` and defines the `Uart` wrapper.
   - Preserves full framework compatibility: `begin(baud)`, `begin(baud, config)`, `end()`, `write()`, `read()`, `available()`, `peek()`, `flush()`, and `operator bool()`.
   - Firmware only interacts with this API and has zero knowledge of simulation queues, fibers, or test harnesses.
   - Forwards all stream calls lazily to its resolved Layer 2 `UartBus`.

2. **Layer 2: Simulation Fabric (`bus/UartBus.h` & `bus/RS485Mux.h`)**:
   - **`UartBus` (Sim-Side Only)**:
     - Exists strictly on the simulation side and models the physical serial data link between UART endpoints.
     - Maintains bidirectional queues (`boost::fibers::buffered_channel<uint8_t> tx_channel_`, `rx_channel_`).
     - **Transaction Peeking & Observability**:
       - Contains logic allowing the simulation side (unit tests, assertions, and packet monitors) to peek at transactions without consuming bytes from the stream.
       - Implements `ITransactionObservable` directly, logging timestamped byte sequences into a circular history buffer.
       - Zero simulation methods are exposed to firmware; they live entirely on `UartBus`.
     - **Per-Bus Snooping & Dedicated Terminal Window Output**:
       - Each `UartBus` instance includes independent snooper configuration:
         ```cpp
         enum class SnoopFormat {
             ASCII,    // Printable text / lines (ideal for CommsSerial debug logs)
             HEX,      // Space-separated hex byte stream: e.g. "5A 01 A0 FF"
             HEX_DUMP  // Formatted hexdump with offset, hex bytes, and ASCII sidebar
         };

         void configure_snooper(SnoopFormat format, std::ostream* output_sink = &std::cout);
         void attach_terminal_window(const std::string& window_title);
         ```
       - **Per-Bus Format Control**:
         - `ASCII`: Pipes text directly or line-buffers output with microsecond timestamps (e.g. for `HW_CommsSerial`).
         - `HEX`: Formats transactions as timestamped hexadecimal streams (`[1250 us] TX: 5A 01 02`).
         - `HEX_DUMP`: Classic logic-analyzer style side-by-side view (offset, hex bytes, and ASCII sidebar).
       - **Independent Terminal Windows (PTY / FIFO / tmux)**:
         - To avoid interleaving rapid RS-485 transactions with ground telemetry logs, each snooped bus can stream to a dedicated terminal.
         - The simulator creates a POSIX pseudo-terminal (`openpty()` allocating `/dev/pts/N`) or named FIFO (`/tmp/toad_uart_<bus_name>`) and can automatically launch an external terminal window (e.g. `x-terminal-emulator`, `gnome-terminal`, `kitty`, or a new `tmux` split pane) attached to that device.
         - This allows developers to observe multiple UART buses running simultaneously in separate, dedicated terminal monitors, mimicking multiple physical FTDI serial monitors on a hardware workbench.
   - **`RS485Mux` (Specialized Transceiver Bus)**:
     - Specializes `UartBus` to model multi-drop RS-485 transceiver routing.
     - Subscribes to Driver Enable (`DE`) and device Select (`SEL`) pin transitions via `SimulatedGPIO` callbacks.
     - When MCU transmits, verifies transceiver state:
       - **TX when DE is LOW**: Logs a warning (`"RS485: Transmit attempted while DE is LOW"`), and discards bytes.
       - **TX with No SEL Active**: Logs a warning (`"RS485: Transmit attempted with no device selected"`).
       - **Multiple SEL Pins Active**: Detects bus contention/collision, corrupts data bytes, and logs a fault.
       - **Valid Routing**: Drops incoming bytes into the targeted device's private channel (`boost::fibers::buffered_channel<uint8_t>`).

3. **Layer 3: Peripheral Models (`peripherals/AMT242AV_Sim.h`)**:
   - Each simulated peripheral fiber owns private input and output queues connected to the `UartBus` / `RS485Mux`.
   - The peripheral fiber loop suspends on `rx_channel_.pop(byte)`.
   - When woken, it decodes position read commands, respects `delayMicroseconds(70)` timing specifications, and pushes encoded position responses into the return channel.

---

### 3.4 CAN Emulation (`VirtualCAN` & `SystemCAN`)

#### Real Hardware Context
Toad implements two CAN topologies ([`lib/can_bus/toad_can_bus.h`](../lib/can_bus/toad_can_bus.h#L7-L18)):
1. **Engine Control CAN Bus (CAN 2.0)**: Connects EC exclusively to TVC actuators and MksServo57D stepper motors.
2. **Vehicle CAN Bus (CAN-FD)**: Connects EC to Flight Controller, Power Board, and GSE.

#### The 3-Layer CAN Architecture
To maintain a strict separation between firmware code, bus transport, and peripheral models, CAN emulation is organized into three distinct layers:

```
 ┌──────────────────────────────────────────────────────────────┐
 │ Layer 1: Firmware API (hal_mock/MockCAN.h)                   │
 │   - Class CAN implementing arduino::CAN                      │
 │   - What ec_main.cpp and MksServo57D call to send/read       │
 └──────────────────────────────┬───────────────────────────────┘
                                │ forwards frames
                                ▼
 ┌──────────────────────────────────────────────────────────────┐
 │ Layer 2: Simulation Fabric (bus/CANBus.h)                    │
 │   - Class CANBus (sim-side only)                             │
 │   - Maintains std::vector<Channel<can_frame_t>*> subscribers │
 │   - broadcast(frame): Pushes frame to every queue            │
 │     (automatically waking up their fibers)                   │
 │   - Logs to ITransactionObservable for future UI             │
 └──────────────┬───────────────────────────────┬───────────────┘
                │ broadcasts                    │ broadcasts
                ▼                               ▼
 ┌──────────────────────────────┐┌──────────────────────────────┐
 │ Layer 3: Peripheral Model A  ││ Layer 3: Peripheral Model B  │
 │ (MksServo57D_Sim)            ││ (TVCActuator_Sim)            │
 │   - Blocks on its own queue  ││   - Blocks on its own queue  │
 │   - Wakes up on broadcast    ││   - Wakes up on broadcast    │
 │   - Does its own filtering   ││   - Does its own filtering   │
 └──────────────────────────────┘└──────────────────────────────┘
```

1. **Layer 1: Firmware API (`hal_mock/MockCAN.h`)**:
   - Implements the standard embedded `arduino::CAN` interface (`begin()`, `write()`, `read()`, `available()`).
   - Firmware only interacts with this API and has zero knowledge of simulation queues or threads.
   - When firmware transmits a frame, it forwards it directly down to the Layer 2 `CANBus`.
   - Incoming frames destined for the MCU are popped from an internal RX queue.

2. **Layer 2: Simulation Fabric (`bus/CANBus.h` / `VirtualCAN`)**:
   - Exists strictly on the simulation side and acts as the physical copper bus medium.
   - Maintains a subscriber list of peripheral queues (`std::vector<boost::fibers::buffered_channel<can_frame_t>*>`).
   - **Broadcast & Wakeup**: When a frame is received from any node (MCU or peripheral), `CANBus::broadcast()` iterates through all registered subscriber queues and pushes a copy of the frame.
   - Pushing into a queue automatically unblocks and wakes up that peripheral's fiber.
   - Implements `ITransactionObservable` to log all broadcast traffic for future UI packet inspection.

3. **Layer 3: Peripheral Models (`peripherals/MksServo_Sim.h`, `TVCActuator_Sim.h`)**:
   - Each simulated peripheral fiber owns its private input queue (`boost::fibers::buffered_channel<can_frame_t>`).
   - The peripheral fiber loop suspends on `rx_queue_.pop(frame)`.
   - **Class-Specific Filtering**: When woken by a broadcast, the peripheral evaluates its own filtering logic:
     ```cpp
     if (frame.can_id != can_id_) {
         continue; // Frame not intended for this device; ignore and return to sleep
     }
     process_device_command(frame);
     ```
   - Filtering responsibility remains entirely encapsulated within the peripheral class, keeping the `CANBus` fabric clean and protocol-agnostic.

4. **`SystemCAN` (Cross-Process Vehicle Bus Placeholder)**:
   - Provides the same C++ interface as `VirtualCAN`.
   - In this initial phase, it acts as a loopback or stub. In Phase 2, it connects to a Linux `SocketCAN` interface (`vcan0`) to bridge across processes to the Flight Controller.

---

### 3.5 Future UI Provisions: Transaction Observability (`ITransactionObservable`)

To support future GUI dashboards and packet monitors (such as Dear ImGui tables or web inspectors), simulated communication interfaces provide a unified mechanism to inspect recent bus transactions.

#### Transaction Data Model & History Buffer
Because a GUI typically refreshes at 60 Hz (every 16.6 ms) while MCU firmware can execute dozens of bus transactions in a single millisecond, storing only the single "last transaction" would cause rapid packet bursts to be overwritten before the UI can render them.

Instead, the observable interface maintains a small circular history buffer (e.g., 64 or 128 entries), while exposing a convenience method for the latest transaction:

```cpp
struct BusTransaction {
    uint64_t timestamp_us;        // Virtual timestamp when transaction completed
    std::vector<uint8_t> tx_data; // Bytes transmitted
    std::vector<uint8_t> rx_data; // Bytes received
    uint32_t channel_or_id;       // CAN ID, SPI CS pin, or RS-485 SEL pin
};

class ITransactionObservable {
public:
    virtual ~ITransactionObservable() = default;

    // Fast query for the latest event (e.g. for UI status badges)
    virtual BusTransaction get_last_transaction() const = 0;

    // Full history query (e.g. for scrolling packet inspector tables)
    virtual const std::deque<BusTransaction>& get_transaction_history() const = 0;
};
```

#### Clean Layer Boundary (No `friend` or Derived Wrapper Classes Required)
MCU firmware code must **never have access to simulation-only methods** like `get_last_transaction()` or queue peeking. Firmware code should only see standard embedded Arduino/HAL methods (`write()`, `read()`, `transfer()`, `available()`).

With the unified 3-layer architecture, this boundary is enforced cleanly by design without resorting to `friend` classes or invasive inheritance workarounds:

```
                      Layer 1: MCU Firmware
                      ┌───────────────────┐
                      │    Uart Object    │ (Instantiated in ec_main.cpp)
                      │ (HardwareSerial)  │
                      └─────────┬─────────┘
                                │ delegates via internal pointer
                                ▼
                      Layer 2: Simulation Fabric
                      ┌───────────────────┐
                      │      UartBus      │
                      │  (Lives in sim)   │
                      └─────────┬─────────┘
                                │ implements
                                ▼
                      ┌───────────────────────────┐
                      │  ITransactionObservable   │ (Queried by UI / Packet Monitor / Assertions)
                      │  - get_last_transaction() │
                      │  - peek_tx_queue()        │
                      │  - get_history()          │
                      └───────────────────────────┘
```

1. **Firmware-Facing Interfaces (`Uart`, `SPIClass`, `CAN`)**:
   Firmware headers in `hal_mock/` declare only standard embedded interfaces. `Uart` inherits from `arduino::HardwareSerial`, `SPIClass` mirrors the Arduino SPI API, and `CAN` implements `arduino::CAN`. Firmware code only ever holds references to these Layer 1 classes. Calling `get_last_transaction()` from firmware is a **compile-time error** because no such method exists in their vtables or classes.

2. **Simulation-Facing Bus Fabrics (`UartBus`, `SPIBus`, `CANBus`)**:
   All transaction history tracking, circular buffers, and stream peeking logic live entirely within the Layer 2 simulation fabrics. Each fabric class publicly inherits from `ITransactionObservable`:
   ```cpp
   class UartBus : public ITransactionObservable {
   public:
       // Standard stream methods invoked by Layer 1 Uart
       virtual size_t write(uint8_t byte);
       virtual int read();
       virtual int available();

       // Simulation / UI Inspection & Peeking
       BusTransaction get_last_transaction() const override;
       const std::deque<BusTransaction>& get_transaction_history() const override;
       int peek_tx_queue() const; // Non-destructive peek at MCU transmit queue
       int peek_rx_queue() const; // Non-destructive peek at peripheral reply queue
   };
   ```

3. **How UI and Test Runners Observe Traffic**:
   The test runner or future UI monitors interact directly with the Layer 2 simulation fabrics (retrieved from `BusRegistry` or runner handles) by querying them as `ITransactionObservable*`:
   ```cpp
   ITransactionObservable* monitor = BusRegistry::get_uart_bus(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE);
   BusTransaction last_tx = monitor->get_last_transaction();
   const auto& history = monitor->get_transaction_history();
   ```

4. **Universal Virtual Logic Analyzer**:
   Because `SPIBus`, `UartBus` (and `RS485Mux`), and `CANBus` all implement `ITransactionObservable`, any future packet inspector UI or test suite assertion engine treats all three communication media identically as generic observable bus endpoints. Zero `friend` declarations or invasive casts are required.

---

### 3.6 Non-Volatile Memory (NVM) & Flash Emulation (Planned Extension)

While secondary to the initial bus and timing milestones, the simulation architecture anticipates storage emulation for drivers currently under development:

#### External QSPI Flash Driver
- **Hardware Context**: [`lib/hardware_mapping/ec_pins.h`](../lib/hardware_mapping/ec_pins.h#L50-L56) defines dedicated Quad-SPI lines (`PIN_QSPI_IO0`..`IO3`, `PIN_QSPI_CLK`, `PIN_QSPI_CS`). A flash driver for this external memory is being actively developed on a separate branch.
- **Simulation Strategy**: The QSPI peripheral model will be backed either by an in-memory byte buffer or an optional host file (`sim_qspi_flash.bin`). This allows testing wear leveling, log dumping, and boot configurations, with the ability to persist storage across test runs.

#### Internal STM32H7 Flash / NVM HAL
- **Hardware Context**: Firmware routines access internal STM32H7 flash sectors for calibration data, board serials, or persistent state flags using the vendor HAL.
- **Simulation Strategy**: We will provide a lightweight mock for the minimal required subset of the STM32H7 HAL flash functions:
  ```cpp
  HAL_StatusTypeDef HAL_FLASH_Unlock(void);
  HAL_StatusTypeDef HAL_FLASH_Lock(void);
  HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
  HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
  ```
  The mock will simulate flash sector erase-before-write semantics and bounds checking without executing real ARM privileged instructions.

---

## 4. Simulated GPIO Subsystem

The GPIO subsystem acts as the physical pin matrix of the microcontroller. Both the bus fabrics (`SPIBus` Chip Selects, `RS485Mux` Select and Driver Enable pins) and simulated peripherals (such as ADC Data Ready lines or valve status feedback) interact through a centralized, thread-safe `SimulatedGPIO` repository.

```
       MCU Firmware Fiber                      Simulated Peripheral Fibers
      ┌────────────────────┐                  ┌───────────────────────────┐
      │  digitalWrite(CS)  │                  │  write(DRDY, LOW)         │
      │  digitalRead(PIN)  │                  │  wait_for_edge(CS, FALL)  │
      └─────────┬──────────┘                  └─────────────┬─────────────┘
                │                                           │
                ▼                                           ▼
   ┌────────────────────────────────────────────────────────────────────────┐
   │                       SimulatedGPIO Singleton                          │
   │                                                                        │
   │  Pin Table: [Pin 0 ... Pin N] (e.g. PA0 to PG15)                       │
   │    ├── State: HIGH / LOW / FLOATING                                    │
   │    ├── Mode: INPUT / OUTPUT / INPUT_PULLUP / ANALOG                    │
   │    ├── Callbacks: std::vector<PinCallback>                             │
   │    └── Fiber Wait Queue: boost::fibers::condition_variable             │
   └───────────────────┬───────────────────────────────────┬────────────────┘
                       │ fires on toggle                   │ unblocks
                       ▼                                   ▼
        ┌──────────────────────────────┐    ┌──────────────────────────────┐
        │  Bus Fabrics                 │    │  Blocked Fibers              │
        │  - SPIBus: on_cs_assert()    │    │  - Peripheral waiting on CS  │
        │  - RS485Mux: update routing  │    │  - Firmware waiting on DRDY  │
        └──────────────────────────────┘    └──────────────────────────────┘
```

### 4.1 Microcontroller Pin Table & `variant_TOAD_H7` Mapping

The GPIO subsystem maintains a direct flat array of all physical pins on the microcontroller, indexed by **Arduino Pin Number**:

#### Pin Number vs. PinName Indexing
- In [`lib/hardware_mapping/ec_pins.h`](../lib/hardware_mapping/ec_pins.h), pins are assigned symbolic names like `#define PIN_ENC_OX_SEL PF10`.
- In [`boards/TOAD_H7/variant_TOAD_H7.h`](../boards/TOAD_H7/variant_TOAD_H7.h), `PF10` expands to integer `90`, and `NUM_DIGITAL_PINS` is defined as `141`.
- When firmware calls `digitalWrite(PIN_ENC_OX_SEL, HIGH)`, the preprocessor resolves this directly to `digitalWrite(90, 1)`. The firmware strictly passes integer pin numbers, never `PinName` enums.
- Therefore, `SimulatedGPIO` tracks pins using a direct flat array:
  ```cpp
  std::array<PinEntry, NUM_DIGITAL_PINS> pin_table_; // 141 entries on TOAD_H7
  ```
  This guarantees instant, branchless O(1) array access without hash maps or translation overhead.

#### Why `PeripheralPins.c` is Not Needed
In STM32duino, `PeripheralPins.c` is only used for hardware alternate-function multiplexing (routing timer channels for PWM, ADC channels, and internal I2C/SPI peripherals). Pure digital GPIO (`digitalRead`, `digitalWrite`, `pinMode`) never uses `PeripheralPins.c`; it is entirely driven by `variant_TOAD_H7.h` and `digitalPin[]` in [`boards/TOAD_H7/variant_TOAD_H7.cpp`](../boards/TOAD_H7/variant_TOAD_H7.cpp).

#### Pin Entry Structure with `PinName` Debug Metadata
While indexing is done by pin number for performance, each `PinEntry` stores `PinName` metadata populated from `variant_TOAD_H7.cpp` for human-readable logs and UI inspector displays:

```cpp
struct PinEntry {
    uint32_t pin_number;     // e.g. 90
    PinName  pin_name;       // e.g. PF_10 (from digitalPin[90] in variant_TOAD_H7.cpp)
    const char* label;       // e.g. "PF10" (for human-readable logging)
    PinState state;          // PIN_LOW, PIN_HIGH, PIN_FLOATING
    PinMode  mode;           // INPUT, OUTPUT, INPUT_PULLUP, etc.
    std::vector<PinCallback> callbacks;
    boost::fibers::condition_variable edge_cv;
};
```

This allows simulation warnings and transaction logs to display readable pin identifiers:
`[1250 us] PF10 (Pin 90): LOW -> HIGH`
rather than ambiguous raw numbers.

---

### 4.2 Simulation-Side Callback Vectors
To avoid tight coupling between the GPIO repository and specific hardware buses, `SimulatedGPIO` provides simulation-only callback hooks:

```cpp
using PinCallback = std::function<void(uint32_t pin, PinState new_state, PinState old_state)>;

void register_pin_callback(uint32_t pin, PinCallback cb);
```

- **Bus Fabric Hooks**:
  - `SPIBus` registers callbacks on all slave CS pins (`PIN_PT_TC_CS_CHAMBER`, etc.). When a pin transitions `HIGH -> LOW`, `SPIBus` asserts the slave device; when it transitions `LOW -> HIGH`, it deasserts the device.
  - `RS485Mux` registers callbacks on `PIN_RS485_6_DE` and channel select pins (`PIN_ENC_OX_SEL`, `PIN_TVC_PITCH_SEL`) to immediately update active transceiver routing.

### 4.3 Fiber Blocking on Pin State Changes (Edge & Level Waiting)
In embedded systems, tasks frequently wait for external hardware signals (e.g. waiting for an ADC conversion complete interrupt on `DRDY`, or a peripheral fiber waiting for its Chip Select to go active).

Rather than busy-spinning and burning virtual clock cycles, `SimulatedGPIO` integrates directly with `boost::fiber` synchronization primitives to allow fibers to block cooperatively until a pin changes:

```cpp
enum class PinEdge { RISING, FALLING, ANY_EDGE };

// Blocks the calling fiber until the requested transition occurs
bool wait_for_edge(uint32_t pin, PinEdge edge, uint64_t timeout_us = 0);

// Blocks the calling fiber until the pin reaches a specific level
bool wait_for_level(uint32_t pin, PinState level, uint64_t timeout_us = 0);
```

#### How Blocking Works with Virtual Time:
1. The calling fiber calls `wait_for_edge(PIN_PT_TC_DRDY, PinEdge::FALLING, 5000)`.
2. The fiber registers its wait condition and suspends on the pin's `condition_variable`.
3. If an optional timeout is specified, a deadline is registered with `VirtualClock`.
4. When another fiber (such as the ADC simulation model) calls `digitalWrite(PIN_PT_TC_DRDY, LOW)`, `SimulatedGPIO` detects the falling edge, notifies the condition variable, and the waiting fiber immediately wakes up and resumes execution.

### 4.4 Bidirectional GPIO Access (`digitalRead` & `digitalWrite`)
Both MCU firmware and peripheral fibers can read and drive GPIO lines:

- **From MCU Firmware**:
  - Firmware calls standard Arduino functions declared in `hal_mock/Arduino.h`:
    ```cpp
    void digitalWrite(uint32_t pin, uint32_t val);
    int digitalRead(uint32_t pin);
    void pinMode(uint32_t pin, uint32_t mode);
    ```
  - These forward directly into `SimulatedGPIO::instance()`.
- **From Peripheral Simulation Fibers**:
  - Peripheral models (e.g. `ADS131M02_Sim`, `AMT242AV_Sim`) use the same routines to drive response pins, simulate hardware alarms, or detect select line activations.
  - When a peripheral fiber modifies an input pin, it simulates external physical signals entering the microcontroller.

### 4.5 Clean Interface Boundary
- **MCU Firmware View**: Includes only `hal_mock/Arduino.h`. Firmware sees only the standard Arduino API (`digitalRead`, `digitalWrite`, `pinMode`).
- **Simulation View**: Simulation fabrics, peripheral models, and the runner include `core/SimulatedGPIO.h`, giving them access to `register_pin_callback()`, `wait_for_edge()`, `wait_for_level()`, and pin table inspection.

---

## 5. Proposed Folder & File Structure

All simulation files reside in `firmware/offline_test/`:

```
firmware/
├── platformio.ini               # Production build (untouched)
├── src/                         # Production sources (untouched)
├── lib/                         # Production libraries (untouched)
└── offline_test/
    ├── README.md                # This document
    ├── CMakeLists.txt           # Build definitions for the test runner
    ├── CMakePresets.json        # Standard configure and build presets
    ├── vcpkg.json               # Manifest declaring boost-fiber and boost-context
    │
    ├── scripts/                 # Automation & setup scripts
    │   └── setup_env.sh         # Helper to bootstrap vcpkg and verify host tools
    │
    ├── hal_mock/                # Shadow headers replacing embedded toolchain
    │   ├── Arduino.h            # millis(), micros(), delay(), digitalWrite(), pinMode()
    │   ├── SPI.h                # Mock SPIClass (wrapper)
    │   ├── HardwareSerial.h     # Mock Uart (inherits arduino::HardwareSerial)
    │   ├── USBSerial.h          # Mock USBSerial (with console output)
    │   ├── MockCAN.h            # Mock CAN driver interface
    │   └── stm32h7xx_hal_flash.h# Mock internal NVM/Flash HAL functions
    │
    ├── core/                    # Core simulation infrastructure
    │   ├── VirtualClock.h/.cpp  # Discrete-event virtual clock
    │   ├── SimulatedGPIO.h/.cpp # 141-pin table, callbacks, wait_for_edge
    │   ├── BusRegistry.h/.cpp   # Maps constructor pins to simulated buses
    │   └── FiberScheduler.h     # Boost.Fiber priority scheduler setup
    │
    ├── bus/                     # Bus fabrics & observability
    │   ├── ITransactionObservable.h # Vtable interface for UI transaction history
    │   ├── UartBus.h/.cpp       # Link & queue manager with peeking (implements ITransactionObservable)
    │   ├── UartSnooper.h/.cpp   # ASCII/HEX/HEX_DUMP formatting & PTY/FIFO terminal sinks
    │   ├── SPIBus.h/.cpp        # CS-based SPI router (implements ITransactionObservable)
    │   ├── RS485Mux.h/.cpp      # SEL/DE-based UART multiplexer (specialized UartBus)
    │   ├── CANBus.h/.cpp        # In-process multi-drop CAN bus (implements ITransactionObservable)
    │   └── SystemCAN.h/.cpp     # Cross-process SocketCAN bridge placeholder
    │
    ├── peripherals/             # Models of physical components (fibers)
    │   ├── ADS131M02_Sim.h/.cpp # Pressure sensor ADC model
    │   ├── MAX31856_Sim.h/.cpp  # Thermocouple amplifier model
    │   ├── AMT242AV_Sim.h/.cpp  # Throttle valve RS-485 encoder model
    │   ├── MksServo_Sim.h/.cpp  # Stepper motor CAN model
    │   ├── TVCActuator_Sim.h/.cpp# TVC pitch/yaw actuator CAN model
    │   └── QSPIFlash_Sim.h/.cpp # External QSPI flash model (future extension)
    │
    ├── tests/                   # Automated verification tests
    │   ├── test_virtual_clock.cpp # Clock jumping and fiber priority verification
    │   ├── test_uart_bus.cpp      # Uart / UartBus queue routing verification
    │   └── test_uart_snooper.cpp  # Snooping formats and terminal output verification
    │
    └── runner/
        └── main.cpp             # Harness entry: setups buses, fibers, calls setup()/loop()
```

### 5.1 Environment Setup & Quickstart

To ensure any team member can clone the repository and run the simulation platform immediately, all external C++ dependencies (`boost-fiber`, `boost-context`) are managed hermetically using **vcpkg Manifest Mode**.

#### Step 1: Install Host Prerequisites
The host machine requires a modern C++17 compiler (`g++` >= 10 or `clang++` >= 11), `cmake` (>= 3.20), standard extraction utilities, and **32-bit multilib libraries** to match the target embedded processor's 32-bit memory model:
```bash
# On Debian / Ubuntu:
sudo apt update && sudo apt install -y build-essential cmake git curl tar unzip gcc-multilib g++-multilib
```

> [!NOTE]
> **32-bit Architecture Target**: To match the pointer widths (`sizeof(void*) == 4`), memory layout, and integer alignments of target 32-bit embedded microcontrollers (such as Teensy 4.1 / ARM Cortex-M), the SITL platform compiles as a **32-bit binary** (`-m32`, triplet `x86-linux`).

#### Step 2: Bootstrap vcpkg (One-Time Setup)
If you do not already have `vcpkg` installed on your machine, clone and bootstrap it in your user directory:
```bash
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh -disableMetrics
export VCPKG_ROOT=~/vcpkg

# Persist to your shell profile (recommended):
echo 'export VCPKG_ROOT=~/vcpkg' >> ~/.bashrc
echo 'export VCPKG_DEFAULT_TRIPLET=x86-linux' >> ~/.bashrc
```

*(Alternatively, run the included setup helper script: `bash offline_test/scripts/setup_env.sh` which checks prerequisites and performs this bootstrap automatically).*

#### Step 3: Configure and Build
Dependencies are declared in `offline_test/vcpkg.json`:
```json
{
  "$schema": "https://raw.githubusercontent.com/microsoft/vcpkg-tool/main/docs/vcpkg.schema.json",
  "name": "toad-offline-test",
  "version-string": "0.1.0",
  "dependencies": [
    "boost-fiber",
    "boost-context"
  ]
}
```

You can build using either CMake Presets or the CMake CLI directly:

**Option A: Using CMake Presets (Recommended)**
```bash
cd firmware/offline_test
cmake --preset default
cmake --build build -j$(nproc)
```

**Option B: Using CMake CLI Directly**
```bash
cd firmware/offline_test
cmake -B build -S . -DVCPKG_TARGET_TRIPLET=x86-linux -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake
cmake --build build -j$(nproc)
```

> [!NOTE]
> On the very first build, vcpkg will automatically download, compile, and cache 32-bit `boost-fiber` and `boost-context` in `offline_test/build/vcpkg_installed/` for `x86-linux`. Subsequent builds are fast and reuse the precompiled binaries.

#### Step 4: Running Verification Tests
Run the test suite using `ctest`:
```bash
ctest --test-dir build --output-on-failure
```
Or execute individual test targets directly:
```bash
./build/test_virtual_clock
./build/test_uart_bus
./build/test_rs485_mux
./build/test_spi_bus
```

---

## 6. Verification & Milestones

1. **Milestone 1: Virtual Clock & Fiber Framework** [COMPLETE]:
   - Implement `VirtualClock` discrete-event deadline jumping (`delay()`, `delayMicroseconds()`).
   - Configure `boost::fiber` custom scheduler with priority properties (`PRIO_FIRMWARE`, `PRIO_SENSORS`, `PRIO_ACTUATOR_PHYSICS`).
   - Verify time advances deterministically without spinning host CPU cycles and that higher-priority tasks run first on simultaneous wakeups. Verified with `test_virtual_clock`.

2. **Milestone 2: Mock UART, UartBus Emulation & Interactive Terminal** [COMPLETE]:
   - Implement Layer 1 `Uart` (`hal_mock/HardwareSerial.h/.cpp`) inheriting from `arduino::HardwareSerial` with `USBSerial` support.
   - Implement Layer 2 `UartBus` (`bus/UartBus.h/.cpp`) with internal bidirectional queues, baud rate timing in `VirtualClock`, and non-destructive sim peeking (`peek_fw_tx()`, `peek_all_fw_tx()`, `get_transaction_history()`).
   - Implement `BusRegistry` (`core/BusRegistry.h/.cpp`) for pin mapping and lazy resolution (`begin()`).
   - Implement `UartSnooper` (`bus/UartSnooper.h/.cpp`) supporting `FORMAT_ASCII`, `FORMAT_HEX`, and `FORMAT_HEX_DUMP`.
   - Implement `UartTerminalHarness` (`runner/UartTerminalHarness.h/.cpp`) and CLI tool (`runner/uart_terminal_harness.cpp`) supporting live console interaction and POSIX pseudo-terminals (`--pty`).
   - Verified 100% via `test_uart_bus`.

3. **Milestone 3: Simulated GPIO, RS-485 Muxing & Pluggable Peripherals (with Pattern 2 Fiber Queues)** [COMPLETE]:
   - Implement `SimulatedGPIO` (`core/SimulatedGPIO.h/.cpp`) providing microsecond pin tracking, digital/analog state management, and edge listener dispatch.
   - Implement `RS485Mux` (`bus/RS485Mux.h/.cpp`) specializing `UartBus` with `SimulatedGPIO` callbacks on Select (`SEL`) lines.
   - Implement abstract `IRS485Device` and lambda-based `FunctionalRS485Device` in `peripherals/IRS485Device.h`, enabling instant 3-line substitution of custom peripheral and motor models.
   - Implement behavioral model `AMT242AV_Sim` (`peripherals/AMT242AV_Sim.h/.cpp`) modeling 12-bit position reporting, zero/reset commands, and hardware 2-bit odd parity calculation.
   - Implement Pattern 2 asynchronous request/response inter-fiber messaging via `boost::fibers::buffered_channel` for `AMT242AV_Sim` and `FunctionalRS485Device`.
   - Implement bus collision/contention detection when multiple `SEL` pins are driven `HIGH` simultaneously (`has_bus_contention()`, `contention_count()`).
   - Verified 100% via `test_rs485_mux` (all 9 test cases passing, including real production driver `firmware/lib/throttle_valves/AMT242AV.cpp` operating concurrently across the fiber boundary).

4. **Milestone 4: UART Transaction Snooping & Dedicated Terminal Output (Deferred)**:
   - Implement independent terminal output sink support (POSIX pseudo-terminal `/dev/pts/N` or named FIFO) allowing each enabled bus to stream live traffic into its own dedicated terminal window.

5. **Milestone 5: SPI Bus, Pluggable Device Emulation & ADS131M02 Model** [COMPLETE]:
   - Implement Layer 1 `SPIClass` (`hal_mock/SPI.h/.cpp`) implementing `arduino::HardwareSPI` with copy-by-value shared bus semantics.
   - Implement Layer 2 `SPIBus` (`bus/SPIBus.h/.cpp`) with synchronous full-duplex transfers, `SimulatedGPIO` CS tracking, bus collision detection on multiple active CS lines, and `SPISettings` clock timing in `VirtualClock`.
   - Implement abstract `ISPIDevice` and lambda-based `FunctionalSPIDevice` in `peripherals/ISPIDevice.h` with `start()`, `stop()`, `join()` fiber lifecycle for concurrent background workers.
   - Implement `ADS131M02_Sim` (`peripherals/ADS131M02_Sim.h/.cpp`) modeling 24-bit simultaneous sampling ADC with background conversion fiber on `PRIO_SENSORS` (5), hardware CCITT-CRC16, and DRDY pin pulsing.
   - Implement `BusRegistry` (`core/BusRegistry.h/.cpp`) SPI pin mapping and lazy resolution (`(mosi, miso, sck)` and named buses).
   - Implement `ISpiObservable` / `ISpiObserver` transaction history logging (`bus/ITransactionObservable.h`).
   - Verified 100% via `test_spi_bus` (all 10 test cases passing, including production driver `firmware/lib/pressure_sensors/ADS131M02.cpp` operating concurrently across the fiber boundary).


6. **Milestone 6: Virtual CAN & Actuator Emulation**:
   - Implement Layer 1 `CAN` (`hal_mock/MockCAN.h`) and Layer 2 `CANBus` (`bus/CANBus.h/.cpp`) multi-drop broadcast fabric.
   - Link [`lib/throttle_valves/ThrottleValves.cpp`](../lib/throttle_valves/ThrottleValves.cpp).
   - Verify motor positioning commands broadcast across `CANBus` to simulated `MksServo57D` and `TVCActuator` models.

7. **Milestone 7: Full EC Loop Run**:
   - Execute [`src/ec_main.cpp`](../src/ec_main.cpp) `setup()` and `loop()`, confirming all modules pass `begin()` checks in simulation.
   - Validate periodic control loop execution and command handling under virtual time.

8. **Milestone 8: Flash & NVM Emulation (Future Extension)**:
   - Link external QSPI flash driver once merged, validating read/write/erase cycles against simulated storage.
   - Verify minimal STM32H7 HAL flash emulation for saving/restoring persistent calibration data.
