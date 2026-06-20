#include "ValveController.h"

namespace ValveController {

bool begin() {
  return true;
}

// TODO - this function
valve_controller_output_t get_controller_output(pressure_readings_t pt_readings, temperature_readings_t tc_readings) {
  valve_controller_output_t vco;
  vco.ox_angle = 0.0;
  vco.fu_angle = 0.0;
  return vco;
}

} // namespace ValveController