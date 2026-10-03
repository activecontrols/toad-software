#pragma once
#include <Arduino.h>

// Interface to the Zucrow test facility through the Zucrow board.
// Fault and sync lines run in both directions, and two analog outputs report
// valve angles to Zucrow's DAQ.

// Valve angle that maps to full scale on the analog outputs. Zucrow configures their scaling to match.
#define FULL_SCALE_ANGLE_DEG 90.0f

namespace ZucrowInterface {

bool begin();

// State of the fault line from Zucrow. FAULT_ASSERTED comes first so a zero-initialized value reads
// as a fault: an unknown state should safe the system, the same as a disconnected line.
enum fault_state_t { FAULT_ASSERTED, FAULT_CLEAR };

// Inputs from Zucrow. These hide pin polarity from callers.
fault_state_t check_fault();
bool check_sync(); // true when Zucrow reports running
// TODO: Rename when di3 and di4 have defined purposes
PinStatus check_di3();
PinStatus check_di4();
// Outputs to Zucrow.
void send_fault();
void send_ok();
void send_sync(bool running);

// Valve angles in degrees. Values outside [0, FULL_SCALE_ANGLE_DEG] are clamped.
void send_valve_angles(float ox_angle_deg, float fu_angle_deg);

void print_status();

} // namespace ZucrowInterface
