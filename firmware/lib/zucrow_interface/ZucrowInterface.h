#pragma once

// Interface to the Zucrow test facility through the Zucrow board.
// Fault and sync lines run in both directions, and two analog outputs report
// valve angles to Zucrow's DAQ.

namespace ZucrowInterface {

bool begin();

// Inputs from Zucrow. Return true when the condition is asserted, so callers
// never deal with pin polarity.
bool check_fault();
bool check_sync();

// Outputs to Zucrow.
void send_fault();
void send_ok();
void send_sync(bool running);

// Valve angles in degrees. Values outside [0, FULL_SCALE_ANGLE_DEG] are clamped.
void send_valve_angles(float ox_angle_deg, float fu_angle_deg);

void print_status();

} // namespace ZucrowInterface
