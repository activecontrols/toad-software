#include "TemperatureSensors.h"
#include "CommandRouter.h"
#include "CommsSerial.h"

#define C_TO_KELVIN 273.15

namespace TemperatureSensors {

Adafruit_MAX31856 tc_chip_1(TC_CHIP_1_SPI_BUS, PIN_TC_CHIP_1_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_2(TC_CHIP_2_SPI_BUS, PIN_TC_CHIP_2_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_3(TC_CHIP_3_SPI_BUS, PIN_TC_CHIP_3_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_4(TC_CHIP_4_SPI_BUS, PIN_TC_CHIP_4_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_5(TC_CHIP_5_SPI_BUS, PIN_TC_CHIP_5_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chip_6(TC_CHIP_6_SPI_BUS, PIN_TC_CHIP_6_CS, MAX31856_TCTYPE_K);
Adafruit_MAX31856 tc_chips[NUM_TC_CHIPS] = {tc_chip_1, tc_chip_2, tc_chip_3, tc_chip_4, tc_chip_5, tc_chip_6};
const char *tc_names[NUM_TC_CHIPS] = {STRINGIFY(TC1), STRINGIFY(TC2), STRINGIFY(TC3),
                                      STRINGIFY(TC4), STRINGIFY(TC5), STRINGIFY(TC6)};
static_assert(NUM_TC_CHIPS == 6);

// Configures each TC chip.
// Always returns true.
bool begin() {
  bool all_chips_connected = true;
  for (size_t i = 0; i < NUM_TC_CHIPS; i++) {
    bool connected = tc_chips[i].begin();
    if (!connected) {
      CommsSerial.printf("TC Board %d failed to connect.", i);
    }
    all_chips_connected &= connected;
  }

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
  tc_readings.TC_5 = tc_chip_5.readThermocoupleTemperature() + C_TO_KELVIN;
  tc_readings.TC_6 = tc_chip_6.readThermocoupleTemperature() + C_TO_KELVIN;
  static_assert(NUM_TC_CHIPS == 6);

  return tc_readings;
}

// Convert Celsius to Fahrenheit.
float c_to_f(float c) {
  return (c * 9.0 / 5.0) + 32;
}

// print TC readings in Fahrenheit.
void print_tc_readings() {
  CommsSerial.print("TC Readings:");
  for (size_t i = 0; i < NUM_TC_CHIPS; i++) {
    CommsSerial.printf("%20s: %6.2f F\n", tc_names[i], c_to_f(tc_chips[i].readThermocoupleTemperature()));
  }
}

} // namespace TemperatureSensors