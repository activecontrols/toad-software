#pragma once

#include "ec_sensors.h"

struct valve_controller_output_t {
  float ox_angle; // deg
  float fu_angle; // deg
};

namespace ValveController {

bool begin();
valve_controller_output_t get_controller_output(pressure_readings_t pt_readings, temperature_readings_t tc_readings);

} // namespace ValveController