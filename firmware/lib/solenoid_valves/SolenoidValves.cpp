#include "SolenoidValves.h"
#include "CommandRouter.h"
#include "CommsSerial.h"

// Solenoid Valves and Ball Valves have hardware differences but are
// both just DOs from a software perspective.

namespace SolenoidValves {

struct sv_bv_t {
  int pin;
  char *name;
  valve_state_t default_state;
};

/* clang-format off */
sv_bv_t sv_and_bvs[NUM_SV_BV_VALVES] = {
    {PIN_SV_DO_1, SV_NAME(1), SV_DEFAULT_STATE(1)},
    {PIN_SV_DO_2, SV_NAME(2), SV_DEFAULT_STATE(2)},
    {PIN_SV_DO_3, SV_NAME(3), SV_DEFAULT_STATE(3)},
    {PIN_SV_DO_4, SV_NAME(4), SV_DEFAULT_STATE(4)},
    {PIN_SV_DO_5, SV_NAME(5), SV_DEFAULT_STATE(5)},
    {PIN_SV_DO_6, SV_NAME(6), SV_DEFAULT_STATE(6)},
    {PIN_SV_DO_7, SV_NAME(7), SV_DEFAULT_STATE(7)},
    {PIN_SV_DO_8, SV_NAME(8), SV_DEFAULT_STATE(8)},
    {PIN_SV_DO_9, SV_NAME(9), SV_DEFAULT_STATE(9)},
    {PIN_BV_DO_10, BV_NAME(10), BV_DEFAULT_STATE(10)},
    {PIN_BV_DO_11, BV_NAME(11), BV_DEFAULT_STATE(11)},
    {PIN_BV_DO_12, BV_NAME(12), BV_DEFAULT_STATE(12)},
    {PIN_BV_DO_13, BV_NAME(13), BV_DEFAULT_STATE(13)},
    {PIN_BV_DO_14, BV_NAME(14), BV_DEFAULT_STATE(14)},
    {PIN_BV_DO_15, BV_NAME(15), BV_DEFAULT_STATE(15)},
    {PIN_BV_DO_16, BV_NAME(16), BV_DEFAULT_STATE(16)}};
static_assert(NUM_SV_BV_VALVES == 16);
/* clang-format on */

bool begin() {
  // On boot, leave all valves in the state they were left in by setting latch
  // enable low.
  digitalWrite(PIN_SV_BV_LATCH_ENABLE, LOW);
  pinMode(PIN_SV_BV_LATCH_ENABLE, OUTPUT);

  for (int i = 0; i < NUM_SV_BV_VALVES; i++) {
    // LOW is the default/safe state for each valve
    digitalWrite(sv_and_bvs[i].pin, LOW);
    pinMode(sv_and_bvs[i].pin, OUTPUT);
  }

  CommandRouter::add(open_valve_by_name, "open_valve", "Opens a solenoid or ball valve. Provide either the P&ID name `SV_N2_01` or the full name `SV_N2_01_rcs_pos_1`.");

  return true;
}

// Converts from VALVE_OPEN / VALVE_CLOSE to LOW / HIGH depending on the valve wiring.
// All valves must enter a 'default / safe' state when they receive a LOW signal.
// This requirement is driven by the flight termination system functionality.
// So to check whether the DO should be LOW or HIGH, just compare against this default state.
bool valve_state_to_logic_level(int valve_num, valve_state_t target_state) {
  return target_state == sv_and_bvs[valve_num].default_state ? LOW : HIGH;
}

// Pulse the latch enable pin, which propagates the current DO states to the valves.
void pulse_latch_enable() {
  // TODO - test this delay / check datasheet
  // TODO - update some saved state
  // TODO - manual command to bypass - `set_valve_state NAME STATE` maybe
  digitalWrite(PIN_SV_BV_LATCH_ENABLE, HIGH);
  delayMicroseconds(50);
  digitalWrite(PIN_SV_BV_LATCH_ENABLE, LOW);
}

// Sets valves from a bitmask of valve states, 1 = OPEN, 0 = CLOSED.
// Bit positions are based on the valve_ids enum.
// TODO - make this accessible as a command from the UI
void set_valves_from_valve_state(uint32_t valve_states) {
  for (int i = 0; i < NUM_SV_BV_VALVES; i++) {
    set_valve_by_num(i, valve_states & (1 << i) ? VALVE_OPEN : VALVE_CLOSE, false);
  }
  pulse_latch_enable();
}

// Set valve by its numerical ID. The valve_ids enum allows using the canonical names with this function.
void set_valve_by_num(int i, valve_state_t state, bool pulse_latch) {
  digitalWrite(sv_and_bvs[i].pin, valve_state_to_logic_level(i, state));
  if (pulse_latch) {
    pulse_latch_enable();
  }
}

// Set valve by its name (either short name or full name). Prints error if name not found.
void set_valve_by_name(const char *name, valve_state_t state) {
  for (int i = 0; i < NUM_SV_BV_VALVES; i++) {
    if (strlen(name) == VALVE_SHORT_NAME_LEN || strncmp(name, sv_and_bvs[i].name, VALVE_SHORT_NAME_LEN) == 0) {
      set_valve_by_num(i, state);
      return;
    }
    if (strcmp(name, sv_and_bvs[i].name) == 0) {
      set_valve_by_num(i, state);
      return;
    }
  }

  CommsSerial.printf("Valve name <%s> not found. Use either the exact full name or P&ID name with underscores.\n", name);
}

// Open valve by its name (either short name or full name). Prints error if name not found.
void open_valve_by_name(const char *name) {
  set_valve_by_name(name, VALVE_OPEN);
}

// Close valve by its name (either short name or full name). Prints error if name not found.
void close_valve_by_name(const char *name) {
  set_valve_by_name(name, VALVE_CLOSE);
}

} // namespace SolenoidValves
