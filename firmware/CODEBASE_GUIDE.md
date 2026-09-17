# TOAD Firmware — Codebase Guide

A from-scratch orientation to `firmware/` for engineers who are new to this repo. Scope: embedded firmware only (`firmware/`) — the UI directory is intentionally excluded. This is a snapshot as of commit `4a25a3b` on branch `features/RS485`; treat any "currently a stub" / "TODO" note as something that may have changed since — check the file before relying on it.

---

## 1. The big picture

This is **one PlatformIO project that builds three separate firmware images** from a shared `src/` and `lib/` tree:

| PlatformIO env | Board | Compiles | Role |
|---|---|---|---|
| `engine_controller` | `TOAD_H7` | `src/ec_main.cpp` (+ any `ec_*.cpp`) | **Engine Controller (EC)** — the main, most-developed firmware. Reads pressure/temperature sensors, drives throttle valves, TVC actuators, RCS thrusters, and solenoid/ball valves. Runs the flight control loop. |
| `flight_controller` | `TOAD_H7` (same board type as EC "for now") | `src/fc_main.cpp` (+ any `fc_*.cpp`) | **Flight Controller (FC)** — currently a stub, not yet implemented. |
| `programmer` | `TOAD_G4` (different, smaller MCU) | `src/prog_main.cpp` (+ any `prog_*.cpp`) | **Programmer board** — a CAN-to-SPI bridge used to remotely flash new firmware onto an EC or FC H7 board over the GSE CAN bus, without a physical debugger. |

This selection happens via `build_src_filter` in [`platformio.ini`](platformio.ini) — each env excludes all `.cpp` files then re-includes only the ones matching its prefix. `lib_ldf_mode = chain+` means PlatformIO only pulls in a `lib/*` folder if something in the active build actually `#include`s it, so each env only compiles the libraries it needs.

**If you're new, start here, in this order:**
1. [`platformio.ini`](platformio.ini) — see what actually gets built.
2. [`lib/hardware_mapping/ec_pins.h`](lib/hardware_mapping/ec_pins.h) — the map of "what's plugged into which pin."
3. [`src/ec_main.cpp`](src/ec_main.cpp) — the entry point that wires everything together.
4. Then dip into whichever `lib/` subsystem you're touching.

---

## 2. `platformio.ini`

Single manifest. A shared `[env]` base plus the three environments above.

```ini
[env]
platform = ststm32 @ 20.0.0
framework = arduino
build_flags =
  -D SERIAL_RX_BUFFER_SIZE=1024 ; increase serial buffer size
  -D SERIAL_TX_BUFFER_SIZE=1024
  -Wl,-u,_printf_float
  -Wl,-u,_scanf_float
  -D USBCON
  -D USBD_USE_CDC
  -D RADIO_BAUD=57600  ; used on all Serial interfaces for consistency, see monitor_speed above
  -O3                  ; compile for speed
```

- `platform = ststm32 @ 20.0.0` is **pinned deliberately** — see [§9, the `Uart`/`HardwareSerial` story](#9-where-is-hardwareserial--uart-defined) for why.
- `RADIO_BAUD=57600` is the one baud rate used consistently across every UART in the firmware.
- Each env adds its own define (`TOAD_ENGINE_CONTROLLER_ONLY`, `TOAD_FLIGHT_CONTROLLER_ONLY`, `TOAD_PROGRAMMER_ONLY`) and its own `build_src_filter`.
- The `flight_controller` env has a TODO: `; TODO - add DFU support, and "upload over CAN" if we are feeling clever.`

---

## 3. `src/` — the three entry points

### `ec_main.cpp` — Engine Controller (the main firmware)

This is the file that owns and wires together nearly every subsystem in `lib/`. Read this file first for the real picture; everything below is a map of what it touches.

**Global hardware objects declared here** (and `extern`-referenced from `ec_pins.h`):

```cpp
CommsSerial_t<USBSerial> USB_CommsSerial;
CommsSerial_t<Uart> HW_CommsSerial(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX);
CommsSerial_t<Uart> HW_FallbackSerial(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX);
// TODO - configure DE pin
Uart RS485_6(PIN_RS485_6_RX, PIN_RS485_6_TX, PIN_RS485_6_DE);
Uart RS485_2(PIN_RS485_2_RX, PIN_RS485_2_TX, PIN_RS485_2_DE);

SPIClass PT_TC_SPI_1(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
SPIClass PT_TC_SPI_3(PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK);
```

Note the live `// TODO - configure DE pin` sitting right above the RS485 UART construction — RS485 direction control via the 3rd constructor arg isn't verified yet.

**Control flow — this is not a plain Arduino sketch.** `loop()` is deliberately minimal:

```cpp
void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}
```

It just pumps bytes from the primary serial into the command router. The real flight logic lives in a separate **`flight_loop()`** function that is registered as a CLI command (`CommandRouter::add(flight_loop, "start_flight_loop")`) and only executes once an operator types that command over serial. `flight_loop()` is a blocking `while(true)`:

1. Resets `kill_flag`/`arm_flag`, drives throttle valves to a 30° starting angle, zeroes TVC, closes RCS.
2. Loop body: drains any pending serial commands (so `kill_flag` can be set mid-flight via a `"k"` command), reads PT/TC sensors, calls `ValveController::get_controller_output()`, and — **only if `arm_flag` is set** — actually commands the throttle valves, TVC actuators, and RCS.
3. On `kill_flag`, breaks out and safes the throttle valves + RCS.

Two `CommandRouter::add_flag` calls wire the CLI commands `"k"` and `"arm"` to `kill_flag`/`arm_flag`, so an operator can arm or abort the flight loop live.

`setup()`: starts all comm serials at `RADIO_BAUD`, RS485 buses at 9600 (`// TODO - what baud?`), SPI buses, waits 3s, prints a startup banner to both `CommsSerial` (macro, see §9 CommsSerial section) and the fallback serial, then calls every subsystem's `begin()` and ANDs the results into `all_modules_ok`. **If any module fails init, the firmware halts in an infinite loop printing an error every 5s rather than proceeding** — a deliberate fail-safe.

Depends on: `CommandRouter.h`, `CommsSerial.h`, `PressureSensors.h`, `RCS.h`, `SolenoidValves.h`, `TVC_Actuators.h`, `TemperatureSensors.h`, `ThrottleValves.h`, `ValveController.h` (which transitively pull in `ec_pins.h`, `ec_sensors.h`, `ec_valves.h`, `toad_can_bus.h`).

### `fc_main.cpp` — Flight Controller (stub, not yet implemented)

```cpp
void setup() 
{
    Serial.begin(9600);
    Serial.println("init"); // had to add this; for some reason without it we don't pull in Print.cpp which causes a linker error because _write is not defined
}
void loop() {}
```

That comment documents a real STM32duino linker quirk: without at least one `Serial.println()` call, `Print.cpp` never gets linked in, and `_write` (used by libc stdio) ends up undefined, breaking the link. The `#include "CommsSerial.h"` at the top is commented out — FC doesn't use the shared comms layer yet.

### `prog_main.cpp` — Programmer board (CAN-to-SPI bootloader bridge)

Sits between the GSE CAN bus and an STM32H7 target (EC or FC board) and can force that H7 into its ST SPI bootloader to flash it without a debugger or USB cable. Explicitly implements ST's app note **AN4286** ("How to use SPI protocol in bootloader on STM32 MCUs") — comments are literally annotated with page citations, e.g. `// Send SYNC byte and get ACK [6]`, `// Mass erase [26]`.

Key protocol constants:
```cpp
constexpr uint8_t ACK = 0x79;
constexpr uint8_t NACK = 0x15;
constexpr uint8_t SYNC = 0x5A;
constexpr uint8_t CMD_WriteMem = 0x31;
constexpr uint8_t CMD_EraseMem = 0x44;
```

Flash paging:
```cpp
// 2 MB of Flash divided into 512 x 4096 byte virtual pages.
// Chunks for the active page are received 32 bytes at a time, for 128 chunks per page.
// The active page is written out 256 bytes at a time.
constexpr size_t CAN_CHUNK_SIZE = 32;
constexpr size_t WRITE_CHUNK_SIZE = 256;
constexpr size_t PAGE_CACHE_SIZE = 4096;
constexpr size_t NUM_PAGE_CACHES = 512;
static_assert(PAGE_CACHE_SIZE * NUM_PAGE_CACHES == 2097152);
```

State machine: `enum prog_state_t { STATE_IDLE, STATE_PRE_ERASE, STATE_READY, STATE_PAGE_SELECTED }`.

One physical programmer image serves **both** EC and FC boards — at boot it reads a GPIO strap pin to decide which target it's talking to: `prog_type = (digitalRead(PIN_PROG_ID) == PROG_ID_FLIGHT_CONTROLLER) ? PROG_FLIGHT_CONTROLLER : PROG_ENGINE_CONTROLLER;`

Key functions: `reset_h7()` (pulses NRST), `spi_ack_frame()` (flagged `// TODO - pretty sure sometimes this has to cycle around until ACK`), `enter_bootloader()`, `erase_memory()`, `write_memory(addr, bytes, len)`.

`loop()` is a CAN-message-driven state machine built on the `CAN_Msg_Decoder` template from `toad_can_bus.h` (see §5). **Known incomplete/buggy areas**: `raw_bytes`/`raw_msg_len` are declared but never actually populated from the CAN peripheral (`// TODO - figure out how to parse the incoming cmd and payload` is left unresolved), and there is a likely real bug where `if (!chunk_rcv)` (an array, always truthy as a pointer) should probably be `if (!chunk_rcv[i])`.

---

## 4. `boards/` — board variant packages (mostly vendor boilerplate)

`boards/TOAD_G4/` and `boards/TOAD_H7/` are STM32duino Arduino-core "variant" packages — largely auto-generated by ST's CubeMX tooling (BSD-3-Clause, `Copyright (c) 2020, STMicroelectronics`, banner comment `Automatically generated from STM32G473R(B-C-E)Tx.xml ... CubeMX DB release 6.0.160`). You generally won't need to edit these, but it helps to know what's in them:

- **`PinNamesVar.h`** — alternate-function pin aliases, wakeup pin mappings, USB D+/D- assignments.
- **`variant_TOAD_*.h`** — numeric pin `#define`s, pin counts, default `LED_BUILTIN`, default SPI/I2C/UART pins, `SERIAL_PORT_*` aliasing macros. `variant_TOAD_H7.h` has one project-specific addition at the end: `// Robert additions` → `#define USE_PWR_LDO_SUPPLY`.
- **`variant_TOAD_*.cpp`** — pin lookup tables plus `SystemClock_Config()`. G4 uses HSI+HSI48 oscillators for USB; H7 uses a separate PLL3 tuned specifically for 48 MHz USB (`PLL3M=32, PLL3N=192, PLL3Q=8`) plus `HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY)`.
- **`PeripheralPins.c`** — ST/CubeMX-generated pin↔peripheral capability tables (ADC, I2C, TIM, UART, SPI, FDCAN, USB, etc.).
- **`ldscript.ld`** — standard linker script; heap/stack sizes reserved via `_Min_Heap_Size`/`_Min_Stack_Size`.

Practical difference between the boards: G4 defaults to `SERIAL_UART_INSTANCE 2`, H7 to `5`; H7 needs the extra `USE_PWR_LDO_SUPPLY`/PLL3-for-USB tweak that G4 doesn't.

---

## 5. `lib/can_bus/toad_can_bus.h` — CAN topology and message formats

Defines the CAN network topology, 11-bit CAN IDs, and every custom CAN wire-format message struct.

Topology, verbatim:
```
// The primary vehicle CAN bus runs from the flight
// controller to the engine controller and includes
// the power management board. This is a CAN-FD bus.

// The engine control CAN bus runs from the engine
// controller to the TVC actuators and the stepper
// drivers. This is a CAN 2.0 bus.

// The GSE CAN bus runs from the GSE, over the QD
// arm and splits to run to each programmer board.
// This is a CAN-FD bus.
```

So there are **three physically distinct CAN networks** — this is why `ec_pins.h` defines both `PIN_CAN_TVC_RX/TX` (engine-control CAN 2.0 bus, to TVC/steppers) and `PIN_CAN_FC_RX/TX` (vehicle CAN-FD bus, to the flight controller).

CAN IDs:
```cpp
// COTS devices (ID range 0x00X)
constexpr uint16_t CAN_ID_TVC_PITCH = 0x001;
constexpr uint16_t CAN_ID_TVC_YAW = 0x002;
constexpr uint16_t CAN_ID_STEPPER_OX = 0x003;
constexpr uint16_t CAN_ID_STEPPER_FU = 0x004;

// Custom boards (ID range 0x01X)
constexpr uint16_t CAN_ID_FLIGHT_CONTROLLER = 0x011;
constexpr uint16_t CAN_ID_ENGINE_CONTROLLER = 0x012;
constexpr uint16_t CAN_ID_FLIGHT_PROG = 0x013;
constexpr uint16_t CAN_ID_ENGINE_PROG = 0x014;
constexpr uint16_t CAN_ID_GSE = 0x015;
constexpr uint16_t CAN_ID_POWER_BOARD = 0x016;
```

Every message struct starts with `uint8_t cmd_id` as a wire discriminator: `can_msg_heartbeat_t` (0x00), error replies `can_msg_invalid_cmd_t`/`can_msg_incorrect_len_t`/`can_msg_unexpected_state_t` (0x01–0x03), telemetry `can_msg_fc_telemetry`/`can_msg_ec_telemetry` (0x10/0x11), and the bootloader protocol messages `can_msg_reset_controller_t`, `can_msg_enter_bootloader_t`, `can_msg_erase_flash_t`, `can_msg_select_page_t`, `can_msg_mem_packet_t`, `can_msg_request_mem_packet_t`, `can_msg_write_flash_t` (0x30–0x36).

**The one real class here:**

```cpp
template <typename state_t> class CAN_Msg_Decoder {
public:
  CAN_Msg_Decoder(const uint8_t *raw_bytes, size_t len, state_t state);
  template <typename msg_t> std::optional<msg_t> decode_and_enforce_state(state_t expected);
  template <typename msg_t> std::optional<msg_t> decode(); // convenience: expected = state
  void send_error_if_not_decoded();
private:
  bool decoded;
  const uint8_t *raw_bytes;
  const size_t len;
  const state_t state;
};
```

Usage pattern (as seen in `prog_main.cpp`):
```cpp
if (const auto msg = raw_msg.decode<can_msg_heartbeat_t>()) { ... }
else if (const auto msg = raw_msg.decode<can_msg_reset_controller_t>()) { ... }
```

`decode_and_enforce_state<msg_t>()` matches `raw_bytes[0]` against `msg_t`'s cmd_id, validates payload length equals `sizeof(msg_t)`, validates the state machine is in the expected state, and on success `memcpy`s into the typed struct.

**Important gap**: none of the error-struct "send" TODOs are implemented anywhere — there is currently **no actual CAN transmit path** wired up in this decoder or its call sites. It prepares error structs locally and drops them.

---

## 6. `lib/hardware_mapping/` — "what's plugged into which pin"

This is the authoritative, human-curated source of pin assignments, separate from the low-level Arduino pin numbering in `boards/`.

### `ec_pins.h`

Organized by peripheral group, each with explanatory comments:
- **UARTs**: `PIN_HW_COMM_SERIAL_RX/TX` = primary serial (UART5), `PIN_HW_FALLBACK_SERIAL_RX/TX` = fallback (UART3).
- **RS485**: `// RS485 Busses (UART6 and UART2)`; `extern Uart RS485_6; extern Uart RS485_2;` (owned by `ec_main.cpp`), plus per-device bus/select macros: `TVC_PITCH_RS485_BUS`/`PIN_TVC_PITCH_SEL`, `ENC_OX_RS485_BUS`/`PIN_ENC_OX_SEL`, yaw/fuel equivalents on `RS485_2`. `DRV_OX_RS485_BUS`/`PIN_DRV_OX_SEL` marked `// unused`.
- **SPI**: two buses (`PT_TC_SPI_1`, `PT_TC_SPI_3`) shared by the PT and TC boards.
- **QSPI**: pins defined, not obviously used yet.
- **FDCAN**: `PIN_CAN_TVC_RX/TX`, `PIN_CAN_FC_RX/TX`.
- **PWM (spark/igniter)**: `PIN_SPARK_PWM`/`PIN_SPARK_TRIG`.
- **"Zucrow Board" pins**: `// TODO - replace with PT board definition` — placeholder for a separate DI/DO interface board (named for Purdue's Zucrow Labs test stand).
- **Valve DOs**: `NUM_SV_BV_VALVES 16`, `PIN_SV_DO_1..9`, `PIN_BV_DO_10..16`, `PIN_VALVE_OE_INPUT`, `PIN_SV_BV_LATCH_ENABLE` (`// TODO - flywire and assign`).
- **PT/TC boards**: `NUM_PT_BOARDS 6` (each = 1 `ADS131M02` driving 2 PT channels), `NUM_TC_CHIPS 6`, split across the two SPI buses.
- **Utility macros**: `CONCAT`/`STRINGIFY` (classic preprocessor token-paste/stringize helpers) used heavily by `ec_sensors.h`/`ec_valves.h` to build canonical names.

### `ec_sensors.h`

Maps numeric PT/TC indices to canonical P&ID-style names, e.g.:
```cpp
#define PT_1 PT_N2_01_tank
#define PT_2 PT_N2_02_reg
...
#define PT_12 PT_FU_05_venturi_upstream
static_assert(NUM_PT_BOARDS == 6);

#define TC_1 TC_N2_01_tank
...
static_assert(NUM_TC_CHIPS == 6);
```

Shared reading structs:
```cpp
struct pressure_readings_t { uint8_t crc_errors; float PT_1..PT_12; }; // all readings in PSI
struct temperature_readings_t { float TC_1..TC_6; }; // all readings in K
```

`PT_CALIBRATION(n)` macro expands to e.g. `PT_1_calibration`, defined in `pt_calibration.h`.

### `ec_valves.h`

Canonical naming + default fail-safe states for all 16 valve DOs:
```cpp
#define VALVE_SHORT_NAME_LEN 8 // "SV_N2_01"
#define SV_1 SV_N2_01_rcs_pos_1
...
#define BV_16 BV_FU_03_run
static_assert(NUM_SV_BV_VALVES == 16);

namespace SolenoidValves {
  enum valve_ids { ... };
  enum valve_state_t { VALVE_CLOSE, VALVE_OPEN };
  // A valve state isn't the same as a DO logic level, so use this type to differentiate.
  // See valve_state_to_logic_level for more info.
}
```

Default positions: N2/O2/FU "release" ball valves default **open** (vent-safe); everything else (RCS, purges, igniters, "run" valves) defaults **closed** (marked `// TODO CONOPS - audit`). Comment in this file: `// Note - this must be kept in sync with the list in SolenoidValves.cpp`.

### `prog_pins.h`

Programmer-board (`TOAD_G4`) pins and the boot-control protocol used to bridge to the H7 target:
```cpp
#define PIN_PROG_ID PB1
#define PROG_ID_FLIGHT_CONTROLLER HIGH
#define PROG_ID_ENGINE_CONTROLLER LOW
enum prog_id_t { PROG_FLIGHT_CONTROLLER, PROG_ENGINE_CONTROLLER };

#define PIN_H7_BOOT PC0
#define BOOT_MODE_RUN LOW
#define BOOT_MODE_FLASH HIGH

#define PIN_H7_NRST PC1
#define NRST_MODE_RUN HIGH
#define NRST_MODE_RST LOW
```
Same firmware image serves both EC and FC — differentiated only by the `PIN_PROG_ID` strap — and directly drives the target H7's BOOT0/NRST pins.

---

## 7. `lib/pressure_sensors/`

### `ADS131M02.h` / `.cpp` — low-level SPI driver

TI ADS131M02, a 2-channel 24-bit delta-sigma ADC used to read pressure transducers. (`// Datasheet: https://www.ti.com/lit/ds/symlink/ads131m02.pdf`, citations marked `[pg#]`.)

```cpp
struct adc_reading_t { uint32_t status_reg; int32_t ch0; int32_t ch1; bool crc_ok; };

class ADS131M02 {
public:
  ADS131M02(SPIClass spi_bus, unsigned int cs_pin);
  void begin();
  adc_reading_t read_adc();
private:
  uint32_t transact_word(uint32_t cmd, uint8_t *crc_buf);
  SPIClass spi_bus;
  unsigned int cs_pin;
};
```

`SPISettings ADS131M02_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE1);` with `// TODO - verify clock integrity with a scope.` `read_adc()` sends the ADC's "NULL" command 4 times (status/ch0/ch1/CRC words per frame), computes a CRC-16 (poly `0x1021`) to validate, and sign-extends the 24-bit two's-complement channel values. Frame protocol comment: `// The ADS131M02 communicates in 24 bit words, grouped into 4 word frames ... This function should be called 4 times to process a complete frame.`

### `PressureSensors.h` / `.cpp` — application layer

```cpp
class PT_Board {
public:
  PT_Board(SPIClass &spi_bus, unsigned int cs_pin, float pt0_slope, float pt0_offset, float pt1_slope, float pt1_offset);
  void begin();
  bool read_pts(float *pt0_reading, float *pt1_reading);
private:
  ADS131M02 adc;
  float pt0_slope, pt0_offset, pt1_slope, pt1_offset;
};

namespace PressureSensors {
  bool begin();
  pressure_readings_t read_pts();
  void print_pt_crc_errors(pressure_readings_t pt_readings);
  void print_pt_readings();
}
```

Instantiates all 6 `PT_Board`s using the `PT_CALIBRATION(n)` macro for slope/offset. `read_pts()` has a live TODO: `// TODO PTs - add preconversion logic to account for voltage level changes / probably need to divide by pow(2, 24) as well` — today's raw-code→engineering-units conversion is a naive linear `slope*raw+offset` and is flagged as incomplete. `begin()` does an initial read; if CRC errors are found at boot it prints them and **returns false**, which propagates into `ec_main.cpp`'s `all_modules_ok` fail-safe halt. Registers CLI command `print_pt`.

### `pt_calibration.h`

Per-sensor linear calibration constants — **currently all placeholders**: every PT is `slope=0, offset=10`. Don't trust any PT reading until these are filled in with real calibration data.

---

## 8. `lib/RS485/` — new, untracked, effectively empty (relevant to your current branch)

This is the directory shown as untracked in `git status` on `features/RS485`.

```cpp
// RS485.h
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

class RS485{
    
}
```

This is a stub: the class body is empty, and **the class definition is missing its terminating semicolon** — `class RS485{ ... }` with no `;` — which will fail to compile if this header is ever `#include`d and the type used. `RS485.cpp` is completely empty (0 bytes).

Nothing else in the codebase references this file yet. Today, RS485 buses are implemented directly as plain `Uart` objects with a manually-toggled `SEL`/DE pin (see `ec_main.cpp`'s `RS485_6`/`RS485_2`, and `AMT242AV`'s hand-rolled RS485 bit-banging below) — this library looks like the start of an effort to factor that pattern out into a reusable class, but it isn't wired into anything yet.

---

## 9. `lib/serial_comms/`

### `CommsSerial.h`

Template wrapper adding buffered `readline()`, `scanf`-style parsing, and `printf`-style convenience methods on top of any Arduino serial-like class (`Uart`, `USBSerial`). Also declares the two shared comm-serial globals used everywhere.

```cpp
#define PRINT_BUFFER_SIZE 1024
#define READ_BUFFER_SIZE 1024

template <typename BaseSerial> class CommsSerial_t : public BaseSerial {
public:
  using BaseSerial::BaseSerial;
  char readbuf[READ_BUFFER_SIZE];
  char *readline();
  template <typename... Args> int scanf(const char *format, Args... args);
  template <typename... Args> void printf(const char *format, Args... args);
  template <typename T> void mprint(const T &t);
  template <typename T, typename... Args> void mprint(const T &t, const Args &...args);
  template <typename... Args> void mprintln(const Args &...args);
};

extern CommsSerial_t<Uart> HW_CommsSerial;
extern CommsSerial_t<USBSerial> USB_CommsSerial;

#define CommsSerial HW_CommsSerial
```

**Important gotcha**: `CommsSerial` is a **macro**, not a variable — `#define CommsSerial HW_CommsSerial`. This is why code all over the tree (`PressureSensors.cpp`, `SolenoidValves.cpp`, etc.) can just write `CommsSerial.println(...)` and have it transparently mean "the primary hardware comm UART." Because it's a preprocessor macro, it can't be locally shadowed or reassigned — keep that in mind if you ever want a function-local variable named `CommsSerial`.

`readline()` busy-waits reading characters until `\n`, with `\b` as a destructive backspace. `printf` snprintfs into a 1 KB stack buffer, then calls `BaseSerial::print()`.

### `CommandRouter.h` / `.cpp`

A newline-delimited ASCII command router over `CommsSerial`, with escaped control characters, used both as an interactive CLI for GSE/test operators and as the registration point for firmware modules' own commands (`print_pt`, `open_valve`, `start_flight_loop`, etc.) and flags (`k`, `arm`).

```cpp
struct command {
  std::function<void(const uint8_t *, size_t)> f;
  const char *name;
  const char *help;
};

#define END_CHAR '\n'
#define CR_CHAR '\r'
#define ESCAPE_CHAR '\\'
#define BACKSPACE_CHAR '\b'

namespace CommandRouter {
  void begin();
  void receive_byte(uint8_t c);
  template <typename S> void send_command(const char *command, S data);
  void help(const char *cmd_name);
  void add(std::function<void(const uint8_t *, size_t)> f, const char *name, const char *help = "no help provided");
  void add(std::function<void(const char *)> fstr, const char *name, const char *help = "no help provided");
  void add(std::function<void()> fvoid, const char *name, const char *help = "no help provided");
  void add_flag(bool *flag, const char *name, const char *help);
}
```

`receive_byte()` is a byte-at-a-time state machine (fed from `ec_main.cpp`'s `loop()`) that builds a command buffer (max `MAX_CMD_LEN 1024`), handles `\n` (dispatch), `\r` (ignored — `// do nothing - we aren't a typewriter, no need to carriage return`), `\b` (backspace), and `\` (escape, so binary payloads like `sv_ui`'s 4-byte bitmask can embed the 4 special bytes literally). `help()` supports `help` (list all), `help <exact_name>`, and `help <prefix>` (prefix match — `// no command found, so print all commands that start with what user entered`). `send_command<S>()` is the outbound side: writes a command name plus a raw binary payload, escaping special bytes, for sending structured telemetry to a host. Built-in commands: `help`, and `ping` (`CommsSerial.println("pong")`, connectivity check).

---

## 10. `lib/solenoid_valves/`

### `SolenoidValves.h` / `.cpp`

Drives the 16 solenoid/ball-valve digital outputs through what's implied to be a latching relay driver (based on the `pulse_latch_enable()` pattern), converting between abstract `VALVE_OPEN`/`VALVE_CLOSE` and the correct GPIO level per valve.

```cpp
namespace SolenoidValves {
  bool begin();
  void pulse_latch_enable();
  void set_valves_from_valve_state(uint32_t valve_states);
  void set_valves_from_valve_state_cmd(const uint8_t *cmd_packet, size_t len);
  void set_valve_by_num(int i, valve_state_t state, bool pulse_latch = true);
  void open_valve_by_name(const char *name);
  void close_valve_by_name(const char *name);
}
```

**Key safety design note — read this if you touch valve code.** Verbatim:
```cpp
// Converts from VALVE_OPEN / VALVE_CLOSE to LOW / HIGH depending on the valve wiring.
// All valves must enter a 'default / safe' state when they receive a LOW signal.
// This requirement is driven by the flight termination system functionality.
// So to check whether the DO should be LOW or HIGH, just compare against this default state.
bool valve_state_to_logic_level(int valve_num, valve_state_t target_state) {
  return target_state == sv_and_bvs[valve_num].default_state ? LOW : HIGH;
}
```
**LOW always means safe**, regardless of whether "safe" happens to be open or closed for a given valve. A wire break, power loss, or FTS (Flight Termination System) trigger naturally drives every valve pin LOW and lands it in its safe configuration.

`begin()`: leaves `PIN_SV_BV_LATCH_ENABLE` LOW at boot (`// On boot, leave all valves in the state they were left in by setting latch enable low.`), configures `PIN_VALVE_OE_INPUT` as input (`// TODO - check state of this pin to see if flight has been terminated`), sets every valve pin LOW+OUTPUT, registers CLI commands `open_valve`, `close_valve`, `sv_ui` (binary bitmask command).

`pulse_latch_enable()` (`// TODO - test this delay / check datasheet`) pulses latch-enable HIGH for 50µs then LOW — this is what actually propagates buffered DO states out to the physical valve driver hardware, implying valve outputs go through a latching driver IC rather than direct GPIO.

### `RCS.h` / `.cpp` — Reaction Control System

Simple bang-bang/deadband control of 4 N2 attitude-thruster solenoid valves.

```cpp
#define RCS_DEADBAND 1 // N?

namespace RCS {
  void close();
  void update_rcs_valves(float rcs_force);
}
```

`update_rcs_valves(rcs_force)`: `>= RCS_DEADBAND` opens "pos" valves + closes "neg"; `<= -RCS_DEADBAND` the reverse; otherwise closes all. Each branch sets 4 valve DOs individually with `pulse_latch=false` then calls `pulse_latch_enable()` once, batching the latch pulse.

---

## 11. `lib/temperature_sensors/`

### `Adafruit_MAX31856.h` / `.cpp`

SPI driver for the MAX31856 thermocouple amplifier, a trimmed fork of Adafruit's library: `// Modifed by Robert Nies to remove dependency on Adafruit_SPIDevice` / `// We only use Hardware SPI on Toad`.

```cpp
class Adafruit_MAX31856 {
public:
  Adafruit_MAX31856(SPIClass &spi_bus, unsigned int cs_pin, max31856_thermocoupletype_t tc_type);
  bool begin(void);
  void setConversionMode(max31856_conversion_mode_t mode);
  max31856_conversion_mode_t getConversionMode(void);
  void setThermocoupleType(max31856_thermocoupletype_t type);
  max31856_thermocoupletype_t getThermocoupleType(void);
  uint8_t readFault(void);
  void triggerOneShot(void);
  bool conversionComplete(void);
  float readCJTemperature(void);
  float readThermocoupleTemperature(void);
  void setTempFaultThreshholds(float flow, float fhigh);
  void setColdJunctionFaultThreshholds(int8_t low, int8_t high);
  void setNoiseFilter(max31856_noise_filter_t noiseFilter);
private:
  SPIClass &spi_bus; unsigned int cs_pin;
  max31856_conversion_mode_t conversionMode;
  max31856_thermocoupletype_t tc_type;
  void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
  uint8_t readRegister8(uint8_t addr);
  uint16_t readRegister16(uint8_t addr);
  uint32_t readRegister24(uint8_t addr);
  void writeRegister8(uint8_t addr, uint8_t reg);
};
```

`SPISettings Adafruit_MAX31856_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE1);` (same unverified-clock TODO as `ADS131M02`). `begin()` enables open-circuit fault detection, zeroes cold-junction offset, sets thermocouple type, enables continuous conversion, and validates connectivity via a register readback of the factory-default `0xC0`. `readThermocoupleTemperature()` reads a 24-bit signed register, sign-extends, shifts off the unused bottom 5 bits, and scales by `0.0078125` (2⁻⁷) to get °C.

**Bug worth knowing about**: `readRegisterN()` and `writeRegister8()` call `pinMode(cs_pin, LOW)` / `pinMode(cs_pin, HIGH)` to toggle chip-select. This should almost certainly be `digitalWrite()`, not `pinMode()` — likely a copy-paste artifact from the upstream Adafruit source. Whether it actually works depends on this STM32duino `pinMode()` implementation tolerating a non-mode second argument, which is fragile. Worth fixing if you're in this file.

### `TemperatureSensors.h` / `.cpp`

```cpp
namespace TemperatureSensors {
  bool begin();
  temperature_readings_t read_tcs();
  void print_tc_readings();
}
```

Instantiates all 6 K-type thermocouple channels across the two shared SPI buses. `#define C_TO_KELVIN 273.15`; `read_tcs()` converts to Kelvin (matches `ec_sensors.h`'s `// all readings in K`).

**Inconsistency worth knowing about**: `print_tc_readings()` (whose own comment says `// print TC readings in Fahrenheit.` and which does call `c_to_f()`) prints using a format string labeled `%6.2f C` — the unit *label* in the printf format is wrong (says C, prints F); the conversion itself is correct.

---

## 12. `lib/throttle_valves/`

### `AMT242AV.h` / `.cpp`

Driver for a CUI AMT242A-V absolute magnetic encoder over half-duplex RS485 (`Uart` + a `SEL` GPIO toggling a MAX485 transceiver between TX/RX), used to read throttle-valve shaft position.

```cpp
class AMT242AV {
public:
  AMT242AV(Uart &uart, unsigned int SEL, uint8_t ID);
  void begin();
  bool read_pos(float *out, int max_retries = 10);
  void zero();
  void reset();
private:
  Uart &uart; unsigned int SEL; uint8_t ID;
  bool wait_for_avail(unsigned long long);
  bool _read_pos(uint16_t *);
};
```

`#define MAX_READING ((1 << 12) - 1)` — 12-bit encoder resolution. `_read_pos()`: clears stale RX bytes, drives `SEL` HIGH (transmit) with a 70µs settle, sends a single-byte read command (`uart.write(ID)`), waits up to 150µs for a 2-byte response, then decodes 14 bits of `transmission` down to a 12-bit position (`// we are using 12 bit encoder, datasheet says to throw out lowest two bits`), and validates an odd-parity checksum in the top 2 bits (comment: `// highest bit is for odd-numbered bits, second highest is for even / checksums calculated using odd parity`). Always leaves `SEL` LOW (idle/receive) before returning. `read_pos()` retries up to `max_tries`, normalizing the raw reading to `[0.0, 1.0]` (`0.0 -> 0 degrees, ..., 1.0 -> 360 degrees`).

Top-of-file TODO: `// TODO - audit code below and update for EC, optionally break out RS485 funcs` — plus dead ISR-based code and a comment (`// flush doesn't do anything on portenta h7 but maybe on other platforms it will`) suggesting this was ported from a prior "Portenta" board and hasn't been fully validated on TOAD hardware yet.

### `MksServo57D.h` / `.cpp`

CAN driver for a Makerbase MKS SERVO57D closed-loop stepper driver, used to actuate throttle valve motors. Cites the datasheet and a reference implementation directly in comments.

```cpp
class MksServo57D {
public:
  MksServo57D(uint16_t can_id) : can_id(can_id) {};
  void begin();
  void set_speed(int16_t speed, uint8_t acceleration = 32);
private:
  template <typename T> std::array<uint8_t, sizeof(T)> to_be_bytes(T value);
  template <size_t N> void send_frame(uint8_t cmd, const std::array<uint8_t, N> &data);
  uint16_t can_id;
};
```

`send_frame<N>()` builds a CAN 2.0 frame `[cmd, ...data, crc]` where `crc = (can_id + cmd + sum(data)) & 0xFF`, but **never actually transmits it** — the function ends with `// TODO - transmit the frame` and does nothing further. **This means `set_speed()` currently has no real hardware effect** — it's fully unconnected to any CAN peripheral driver. `set_speed()` itself clamps `|speed|` to `[0, 400]` RPM (`// Max speed in open loop mode is 400 RPM.`) and packs direction into the top bit.

### `ThrottleValves.h` / `.cpp`

Combines one `MksServo57D` (motor) + one `AMT242AV` (encoder) per throttle valve (ox and fuel).

```cpp
class ThrottleValve {
public:
  ThrottleValve(uint16_t motor_can_id, Uart &enc_uart, unsigned int enc_SEL, unsigned int enc_ID);
  void begin();
  void stop();
  void set_position(float angle);
private:
  MksServo57D motor;
  AMT242AV encoder;
};

namespace ThrottleValves {
  bool begin();
  void stop();
  void set_angles_ox_fu(float ox_angle, float fu_angle);
}
```

`set_position()` is a bare proportional controller (`// TODO - PID controller logic`): `target_speed = (angle - current_angle) * K` with `float K = 1; // TODO - set this constant` — an unset/untuned P-only gain, no I or D term. Also flagged: `// TODO - if this turns the motor on, make sure we don't leave it on by mistake! could use heartbeat to solve`. **`ThrottleValves::begin()` currently just `// TODO - don't just return true here!`** — no real health check, so unlike `PressureSensors`/`TemperatureSensors`, this module can never fail the `all_modules_ok` gate in `ec_main.cpp`.

---

## 13. `lib/tvc_actuators/`

### `GimbalKinematics.h` / `.cpp`

Pure-math library converting a commanded pitch/yaw pair into the two linear actuator extension lengths needed to achieve it, via 3D point rotation.

```cpp
void calc_actuator_lengths(float primary_angle, float secondary_angle, float *primary_length, float *secondary_length);
```

Geometry constants (all marked `// TODO - update these`):
```cpp
#define BASE_POINT_DIST_FROM_ORIGIN_H 6.25
#define BASE_POINT_DIST_FROM_ORIGIN_V -2
#define ENGINE_POINT_DIST_FROM_ORIGIN_H 3.475
#define ENGINE_POINT_DIST_FROM_ORIGIN_V 10.35
#define BASE_ACTUATOR_LEN 12.6
```

Rotates 4 fixed 3D attachment points by pitch then yaw and computes each actuator's required length as a *delta* from its neutral length (`// output lengths in inches (already subtracted from the base actuator length)`). Standalone and currently unverified — placeholder geometry constants mean computed lengths aren't trustworthy until measured on real hardware.

### `TVC_Actuators.h` / `.cpp`

```cpp
namespace TVC_Actuators {
  bool begin();
  void set_angles_pitch_yaw(float pitch, float yaw);
}
```

`begin()` just `return true;`. `set_angles_pitch_yaw()` calls `calc_actuator_lengths()` but then **does nothing with the result** (`// TODO - this function`) — the actual actuator-commanding logic (presumably over CAN to `CAN_ID_TVC_PITCH`/`CAN_ID_TVC_YAW`) hasn't been written yet. This is the least-implemented module in the tree — a pure stub.

---

## 14. `lib/valve_controller/ValveController.h` / `.cpp`

Intended home for the EC's closed-loop throttle control algorithm — meant to take live PT/TC readings and compute target throttle-valve angles. Currently a stub.

```cpp
struct valve_controller_output_t {
  float ox_angle; // deg
  float fu_angle; // deg
};

namespace ValveController {
  bool begin();
  valve_controller_output_t get_controller_output(pressure_readings_t pt_readings, temperature_readings_t tc_readings);
}
```

`get_controller_output()` (`// TODO - this function`) always returns `{0.0, 0.0}` regardless of input — the core engine-mixture/thrust control loop that `ec_main.cpp`'s `flight_loop()` calls every iteration is entirely a placeholder today.

---

## 15. The sensor/actuator abstraction pattern

There's **no shared C++ interface/base class** — no virtual `ISensor`/`IValve`. Instead the codebase follows a consistent two-layer, hand-rolled convention repeated per subsystem:

1. **Low-level driver class** (one per physical chip): `ADS131M02`, `Adafruit_MAX31856`, `AMT242AV`, `MksServo57D`. Each wraps exactly one device, takes its bus/pins in the constructor, exposes `begin()` plus device-specific methods.
2. **Application-level `namespace` singleton** (one per logical subsystem, matching the `ec_*` hardware_mapping headers): `PressureSensors`, `TemperatureSensors`, `SolenoidValves`, `ThrottleValves`, `TVC_Actuators`, `ValveController`. Each owns statically-allocated driver instances for every physical unit, and follows a uniform naming convention:
   - `bool begin()` — inits all owned hardware, returns overall success. Consumed by `ec_main.cpp`'s `all_modules_ok &= X::begin();` gate. **Note**: today this gate is only meaningfully enforced by `PressureSensors` and `TemperatureSensors` — `ThrottleValves`, `TVC_Actuators`, and `ValveController` all just `return true`.
   - a `read_*()`/`get_*()` returning a plain result struct (`pressure_readings_t`, `temperature_readings_t`, `valve_controller_output_t`) — these structs, defined centrally in `ec_sensors.h`/`ValveController.h`, are the shared data-interchange format between subsystems.
   - a `set_*`/action function for actuators (`set_angles_ox_fu`, `set_angles_pitch_yaw`, `update_rcs_valves`, `set_valves_from_valve_state`).
   - most register at least one `CommandRouter::add(...)` CLI command for interactive debugging (`print_pt`, `print_tc`, `open_valve`/`close_valve`/`sv_ui`).

A middle "per-unit" class recurs for multi-channel devices — `PT_Board` (2 PTs sharing 1 ADC) and `ThrottleValve` (1 motor + 1 encoder) — bundling multiple driver instances that logically belong together, instantiated as fixed-size arrays/named globals inside the owning namespace's `.cpp`.

This keeps each module simple but means some duplication (every SPI sensor driver independently defines its own `SPISettings`, every namespace writes its own `begin()`-failure printf loop) — a candidate for a future shared-interface refactor if you're looking for one.

---

## 16. How CAN, RS485, and Serial fit together

- **USB / primary serial (`HW_CommsSerial`, aliased `CommsSerial`) and fallback serial (`HW_FallbackSerial`)** — the human/GSE-facing text CLI, driven by `CommandRouter`. Used to arm/kill the flight loop, open/close valves manually, read live sensor values. **Not** part of the real-time flight-critical path — polled non-blockingly once per `loop()`/`flight_loop()` iteration.
- **RS485 (`RS485_6`, `RS485_2`, plain `Uart` + DE pin)** — half-duplex point-to-point links used specifically for the `AMT242AV` absolute encoders (plus reserved-but-unused `DRV_OX_RS485_BUS`/`DRV_FU_RS485_BUS` slots for driver comms). This is the *sensor feedback* bus for throttle-valve position and TVC actuator selection, addressed via per-device `SEL` GPIOs so multiple devices can share one physical bus.
- **CAN (FDCAN peripherals, `toad_can_bus.h`)** — the command/actuation bus. Connects EC to the `MksServo57D` stepper drivers and TVC actuators on the "engine control CAN bus" (CAN 2.0), and separately EC↔FC↔power-board on the "primary vehicle CAN bus" (CAN-FD) and GSE↔programmer-boards on the "GSE CAN bus" (CAN-FD). `CAN_Msg_Decoder` standardizes parsing incoming frames against a state machine — used concretely today in `prog_main.cpp`'s bootloader protocol.

**Gap to know about**: the actual CAN *transmit* path — whether from `MksServo57D::send_frame()` or from `CAN_Msg_Decoder`'s error-reply TODOs — isn't implemented anywhere yet. The CAN layer is currently receive/decode-only in terms of what's actually wired up, even though the send-side framing logic already exists.

**In short**: serial = human interactive control/telemetry · RS485 = local sensor feedback (encoders) · CAN = inter-board command/actuation and cross-controller telemetry/bootloading. Three physically and functionally distinct layers.

---

## 17. Where is `HardwareSerial` / `Uart` defined?

You asked specifically about this — here's the story. `HardwareSerial`/`Uart` are **not part of this repo**; they come from the STM32duino Arduino core, bundled inside the PlatformIO `ststm32` platform package (`framework-arduinoststm32`). That package hasn't been downloaded in this checkout (no `.pio` build dir yet, meaning the project hasn't been built here) — so there's no local file path to point you to until you run a build (`pio run -e engine_controller`), which will fetch it.

Once fetched, it'll land under something like:
```
~/.platformio/packages/framework-arduinoststm32/cores/arduino/HardwareSerial.h
~/.platformio/packages/framework-arduinoststm32/cores/arduino/stm32/uart.c   (C HAL wrapper)
~/.platformio/packages/framework-arduinoststm32/libraries/SrcWrapper/src/stm32/uart.cpp
```
(not independently verified against this checkout — verify once you've built).

**Why `Uart` and not `HardwareSerial`?** This repo's own git history explains it — three relevant commits:
```
84d8c1b switch to ststm32 platform version 20.0.0 change all uses of HardwareSerial to Uart due to breaking change on stm32duino version 3.0.0
bf39458 pin platform due to breaking HardwareSerial changes
d8a0580 make the linker error go away that is caused by missing _write definition
```
STM32duino's core made a breaking change in v3.0.0: `HardwareSerial` became the abstract/base API class (inherited from ArduinoCore-API), and `Uart` became STM32's concrete subclass that application code should instantiate directly. `platformio.ini` pins `platform = ststm32 @ 20.0.0` specifically to lock in this version of the core. Every UART object in this codebase — `ec_main.cpp`'s `HW_CommsSerial`, `HW_FallbackSerial`, `RS485_6`, `RS485_2`; `ec_pins.h`'s `extern Uart RS485_6;`; `AMT242AV`'s `Uart &uart` constructor arg — consistently uses `Uart`, confirming the migration was applied throughout.

---

## 18. Known stubs / TODOs / bugs worth knowing before you dive in

A consolidated list, so you don't have to rediscover these:

| Location | Issue |
|---|---|
| `lib/RS485/RS485.h` | Empty stub class, missing terminating `;`, not referenced anywhere. |
| `lib/tvc_actuators/TVC_Actuators.cpp` | `set_angles_pitch_yaw()` computes lengths but does nothing with them — pure stub. |
| `lib/valve_controller/ValveController.cpp` | `get_controller_output()` always returns `{0, 0}` — the core throttle control loop is unimplemented. |
| `lib/throttle_valves/MksServo57D.cpp` | `send_frame()` builds the CAN frame but never transmits it — `set_speed()` has no hardware effect. |
| `lib/throttle_valves/ThrottleValves.cpp` | `begin()` always returns `true` — no real health check, unlike PressureSensors/TemperatureSensors. |
| `lib/throttle_valves/ThrottleValves.cpp` | `set_position()` is P-only control with an untuned `K = 1`; no PID yet. |
| `lib/can_bus/toad_can_bus.h` | `CAN_Msg_Decoder`'s error-reply paths are all `// TODO - send` — no CAN transmit path implemented anywhere. |
| `lib/temperature_sensors/Adafruit_MAX31856.cpp` | Chip-select toggled via `pinMode(cs_pin, HIGH/LOW)` instead of `digitalWrite()` — likely a bug carried from upstream Adafruit code. |
| `lib/temperature_sensors/TemperatureSensors.cpp` | `print_tc_readings()` prints Fahrenheit values but labels them `C` in the format string. |
| `lib/pressure_sensors/pt_calibration.h` | All PT calibration constants are placeholders (`slope=0, offset=10`). |
| `lib/pressure_sensors/PressureSensors.cpp` | Raw ADC→engineering-units conversion flagged as incomplete (missing voltage-level/2²⁴ scaling). |
| `lib/tvc_actuators/GimbalKinematics.cpp` | All actuator geometry constants marked `// TODO - update these` — computed lengths not yet trustworthy. |
| `src/prog_main.cpp` | CAN RX payload parsing (`raw_bytes`/`raw_msg_len`) never actually populated; likely `chunk_rcv[i]` vs `chunk_rcv` array-truthiness bug in the page-write state machine. |
| `lib/throttle_valves/AMT242AV.cpp` | Header flags `// TODO - audit code below and update for EC` — ported from a prior "Portenta" board, not fully validated on TOAD hardware. |
| `src/fc_main.cpp` | Entire FC firmware is a two-line stub. |

---

*Generated as an orientation aid — verify against the current source before relying on specifics, since this is a fast-moving embedded codebase and several of the "stub"/"TODO" items above are exactly the kind of thing that gets fixed without this doc being updated.*
