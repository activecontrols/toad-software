#pragma once

#include "Adafruit_MAX31856.h"
#include "ec_pins.h"
#include "ec_sensors.h"

namespace TemperatureSensors {

bool begin();
temperature_readings_t read_tcs();

} // namespace TemperatureSensors