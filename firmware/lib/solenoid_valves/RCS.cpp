#include "RCS.h"
#include "SolenoidValves.h"

#define RCS_DEADBAND 1 // N?

namespace RCS {

void reset() {
  SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
  SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
  SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
  SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
  SolenoidValves::pulse_latch_enable();
}

void update_rcs_valves(float rcs_force) {
  if (rcs_force >= RCS_DEADBAND) {
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_OPEN, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_OPEN, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::pulse_latch_enable();
  } else if (rcs_force <= -RCS_DEADBAND) {
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_OPEN, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_OPEN, false);
    SolenoidValves::pulse_latch_enable();
  } else {
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::set_valve_by_num(SolenoidValves::SV_N2_01_rcs_pos_1, SolenoidValves::VALVE_CLOSE, false);
    SolenoidValves::pulse_latch_enable();
  }
}

} // namespace RCS