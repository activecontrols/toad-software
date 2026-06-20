#pragma once
#include "ec_valves.h"

namespace SolenoidValves {

bool begin();

void pulse_latch_enable();
void set_valves_from_valve_state(uint32_t valve_states);
void set_valve_by_num(int i, valve_state_t state, bool pulse_latch = true);
void open_valve_by_name(const char *name);
void close_valve_by_name(const char *name);

} // namespace SolenoidValves