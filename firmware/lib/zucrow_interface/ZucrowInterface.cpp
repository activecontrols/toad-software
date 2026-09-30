#include "ZucrowInterface.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "ec_pins.h"

// Analog chain on the Zucrow board:
//   MCP4822 12 bit DAC, internal 2.048 V reference, 1x gain
//   -> TLV9142 non-inverting stage, gain = 1 + 15k / 5k = 4
//   -> AO_1 (ox) and AO_2 (fu) at the Zucrow connector
// Full scale is 2.048 V * 4 = 8.192 V, which is exactly 2 mV per count.
#define DAC_COUNTS 4096
#define DAC_MAX_COUNT (DAC_COUNTS - 1)
#define VOLTS_PER_COUNT (2.048f * 4.0f / DAC_COUNTS)

// MCP4822 write command is one 16 bit word [MCP4822 datasheet, write command register]:
//   bit 15 channel (0 = A, 1 = B), bit 13 gain (1 = 1x), bit 12 output (1 = active), bits 11:0 data
// LDAC is tied to GND_ISO on this board, so each write updates the output immediately.
#define MCP4822_CHANNEL_A 0x0000
#define MCP4822_CHANNEL_B 0x8000
#define MCP4822_GAIN_1X 0x2000
#define MCP4822_ACTIVE 0x1000

// TODO - verify clock integrity with a scope. The ISO6441 isolator is rated to 150 Mbps, so the
// limit here is the long traces to the Zucrow board. Matches the other devices on this bus.
SPISettings ZUCROW_DAC_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE0);

// Line assignments. DI3 and DI4 are reserved for the Skipper autosequences.
// TODO - confirm DI1/DI2 and DO1/DO2 against the schematic net names.
#define PIN_ZUCROW_FAULT_IN PIN_ZUCROW_BOARD_DI1
#define PIN_ZUCROW_SYNC_IN PIN_ZUCROW_BOARD_DI2
#define PIN_EC_FAULT_OUT PIN_ZUCROW_BOARD_DO1
#define PIN_EC_SYNC_OUT PIN_ZUCROW_BOARD_DO2

// Line polarities, mirroring TADPOLE: Zucrow pulls its lines low to assert, we drive ours high.
// TODO - verify on the bench that an unplugged Zucrow connector reads as a fault. A disconnected
// line must safe the system.
#define ZUCROW_FAULT_LEVEL LOW
#define ZUCROW_RUNNING_LEVEL LOW
#define EC_FAULT_LEVEL HIGH
#define EC_OK_LEVEL LOW
#define EC_RUNNING_LEVEL HIGH
#define EC_IDLE_LEVEL LOW

namespace ZucrowInterface {

uint16_t last_count_ox;
uint16_t last_count_fu;

void write_dac(uint16_t channel, uint16_t count) {
  // beginTransaction before CS so the bus is in this device's SPI mode before it is selected.
  ZUCROW_BOARD_SPI_BUS.beginTransaction(ZUCROW_DAC_SPI_SETTINGS);
  digitalWrite(PIN_ZUCROW_BOARD_CS, LOW);
  ZUCROW_BOARD_SPI_BUS.transfer16(channel | MCP4822_GAIN_1X | MCP4822_ACTIVE | count);
  digitalWrite(PIN_ZUCROW_BOARD_CS, HIGH);
  ZUCROW_BOARD_SPI_BUS.endTransaction();
}

uint16_t angle_to_count(float angle_deg) {
  // Written as !(x > 0) rather than x <= 0 so that NaN also maps to 0 instead of
  // being cast to an integer, which is undefined behavior.
  if (!(angle_deg > 0.0f)) {
    return 0;
  }

  // Full scale maps to 4096, one past what a 12 bit DAC can hold.
  float count = angle_deg / FULL_SCALE_ANGLE_DEG * DAC_COUNTS;
  if (count > DAC_MAX_COUNT) {
    return DAC_MAX_COUNT;
  }

  return (uint16_t)(count + 0.5f);
}

void send_valve_angles(float ox_angle_deg, float fu_angle_deg) {
  last_count_ox = angle_to_count(ox_angle_deg);
  last_count_fu = angle_to_count(fu_angle_deg);
  write_dac(MCP4822_CHANNEL_A, last_count_ox);
  write_dac(MCP4822_CHANNEL_B, last_count_fu);
}

bool check_fault() {
  return digitalRead(PIN_ZUCROW_FAULT_IN) == ZUCROW_FAULT_LEVEL;
}

bool check_sync() {
  return digitalRead(PIN_ZUCROW_SYNC_IN) == ZUCROW_RUNNING_LEVEL;
}

void send_fault() {
  digitalWrite(PIN_EC_FAULT_OUT, EC_FAULT_LEVEL);
}

void send_ok() {
  digitalWrite(PIN_EC_FAULT_OUT, EC_OK_LEVEL);
}

void send_sync(bool running) {
  digitalWrite(PIN_EC_SYNC_OUT, running ? EC_RUNNING_LEVEL : EC_IDLE_LEVEL);
}

void print_status() {
  CommsSerial.printf("From Zucrow: fault %s, sync %s\n", check_fault() ? "ASSERTED" : "clear",
                     check_sync() ? "running" : "idle");
  CommsSerial.printf("To Zucrow:   fault %s, sync %s\n",
                     digitalRead(PIN_EC_FAULT_OUT) == EC_FAULT_LEVEL ? "ASSERTED" : "clear",
                     digitalRead(PIN_EC_SYNC_OUT) == EC_RUNNING_LEVEL ? "running" : "idle");
  CommsSerial.printf("OX: %4u counts, %.3f V\n", last_count_ox, last_count_ox * VOLTS_PER_COUNT);
  CommsSerial.printf("FU: %4u counts, %.3f V\n", last_count_fu, last_count_fu * VOLTS_PER_COUNT);
}

void send_angles_cmd(const char *args) {
  float ox_angle_deg;
  float fu_angle_deg;
  if (sscanf(args, "%f %f", &ox_angle_deg, &fu_angle_deg) != 2) {
    CommsSerial.println("Usage: zi_angles <ox_deg> <fu_deg>");
    return;
  }
  send_valve_angles(ox_angle_deg, fu_angle_deg);
  print_status();
}

// Steps both outputs through known counts, holding each long enough to read a multimeter
// at the Zucrow connector. The readings should land on 2 mV per count.
void calibration_sweep() {
  const uint16_t counts[] = {0, 1024, 2048, 3072, DAC_MAX_COUNT};
  for (uint16_t count : counts) {
    last_count_ox = count;
    last_count_fu = count;
    write_dac(MCP4822_CHANNEL_A, count);
    write_dac(MCP4822_CHANNEL_B, count);
    CommsSerial.printf("%4u counts, expect %.3f V\n", count, count * VOLTS_PER_COUNT);
    delay(5000);
  }
  send_valve_angles(0.0f, 0.0f);
  CommsSerial.println("Sweep done, outputs zeroed.");
}

bool begin() {
  // Deselect the DAC first. SPI1 is shared with PT and TC boards, and until this runs the CS pin
  // floats, which could let the DAC clock in their traffic.
  // pinMode must come before digitalWrite on every output here: stm32duino only enables the GPIO
  // port clock inside pinMode, so an earlier write is silently dropped and the pin comes up LOW.
  pinMode(PIN_ZUCROW_BOARD_CS, OUTPUT);
  digitalWrite(PIN_ZUCROW_BOARD_CS, HIGH);

  pinMode(PIN_ZUCROW_FAULT_IN, INPUT);
  pinMode(PIN_ZUCROW_SYNC_IN, INPUT);

  // Boot reporting a fault, and only report ok once we are actually ready. A controller that
  // resets or hangs during boot then looks like a fault to Zucrow. Same as TADPOLE.
  pinMode(PIN_EC_FAULT_OUT, OUTPUT);
  digitalWrite(PIN_EC_FAULT_OUT, EC_FAULT_LEVEL);
  pinMode(PIN_EC_SYNC_OUT, OUTPUT);
  digitalWrite(PIN_EC_SYNC_OUT, EC_IDLE_LEVEL);

  // No DAC write here. The MCP4822 powers up with both outputs shut down, which already reads as
  // 0 V at the connector, and the first write to each channel enables it. Skipping the write means
  // begin() puts no traffic on SPI1 while other devices' CS pins may still be floating.

  /* clang-format off */
  CommandRouter::add(print_status, "zi_status", "Print Zucrow line states and analog outputs.");
  CommandRouter::add(send_fault, "zi_fault", "Assert the fault line to Zucrow.");
  CommandRouter::add(send_ok, "zi_ok", "Clear the fault line to Zucrow.");
  CommandRouter::add([]() { send_sync(true); }, "zi_run", "Set the sync line to Zucrow to running.");
  CommandRouter::add([]() { send_sync(false); }, "zi_idle", "Set the sync line to Zucrow to idle.");
  CommandRouter::add(send_angles_cmd, "zi_angles", "Send valve angles to Zucrow. Args: <ox_deg> <fu_deg>");
  CommandRouter::add(calibration_sweep, "zi_sweep", "Step the analog outputs through known counts for calibration.");
  /* clang-format on */

  // There is no readback path on this board, so nothing here can fail. The calibration sweep
  // is what proves the analog chain works.
  return true;
}

} // namespace ZucrowInterface
