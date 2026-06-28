#include "TemperatureSensors.h"
#include "CommandRouter.h"
#include "CommsSerial.h"

#define C_TO_KELVIN 273.15

namespace TemperatureSensors {

Adafruit_MAX31856 tc_chip_1(TC_BOARD_1_SPI_BUS, PIN_TC_BOARD_1_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_2(TC_BOARD_1_SPI_BUS, PIN_TC_BOARD_2_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_3(TC_BOARD_1_SPI_BUS, PIN_TC_BOARD_3_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_4(TC_BOARD_1_SPI_BUS, PIN_TC_BOARD_4_CS, MAX31856_TCTYPE_K);
static_assert(NUM_TC_CHIPS == 4);

// Configures each TC chip.
// Always returns true.
bool begin() {
  bool all_chips_connected = true;
  all_chips_connected &= tc_chip_1.begin();
  all_chips_connected &= tc_chip_2.begin();
  all_chips_connected &= tc_chip_3.begin();
  all_chips_connected &= tc_chip_4.begin();

  CommandRouter::add(print_tc_readings, "print_tc", "Print TC readings in Fahrenheit.");

  return all_chips_connected;
}

// Read pressure value from all PTs, recording CRC errors if they occur.
temperature_readings_t read_tcs() {
  temperature_readings_t tc_readings;

  // TODO - for performance, consider parallelizing across the busses (only if needed)
  // TODO - read fault reg and sanity check values
  tc_readings.TC_1 = tc_chip_1.readThermocoupleTemperature() + C_TO_KELVIN;
  tc_readings.TC_2 = tc_chip_2.readThermocoupleTemperature() + C_TO_KELVIN;
  tc_readings.TC_3 = tc_chip_3.readThermocoupleTemperature() + C_TO_KELVIN;
  tc_readings.TC_4 = tc_chip_4.readThermocoupleTemperature() + C_TO_KELVIN;

  return tc_readings;
}

// Convert Celsius to Fahrenheit.
float c_to_f(float c) {
  return (c * 9.0 / 5.0) + 32;
}

// print TC readings in Fahrenheit.
void print_tc_readings() {
  CommsSerial.print("TC Readings:");
  CommsSerial.printf("%20s: %6.2f C\n", STRINGIFY(TC_1), c_to_f(tc_chip_1.readThermocoupleTemperature()));
  CommsSerial.printf("%20s: %6.2f C\n", STRINGIFY(TC_2), c_to_f(tc_chip_2.readThermocoupleTemperature()));
  CommsSerial.printf("%20s: %6.2f C\n", STRINGIFY(TC_3), c_to_f(tc_chip_3.readThermocoupleTemperature()));
  CommsSerial.printf("%20s: %6.2f C\n", STRINGIFY(TC_4), c_to_f(tc_chip_4.readThermocoupleTemperature()));
}

} // namespace TemperatureSensors