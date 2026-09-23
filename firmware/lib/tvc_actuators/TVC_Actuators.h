#pragma once

// Destination in repo: firmware/lib/tvc_actuators/TVC_Actuators.h
// (adds poll() - everything else unchanged from the existing file)

namespace TVC_Actuators {

bool begin();
void set_angles_pitch_yaw(float pitch, float yaw);

// Drains and processes any actuator feedback waiting on the Actuators CAN
// bus. Must be called from flight_loop() (or loop()) for feedback to ever be
// seen - nothing does this automatically.
void poll();

} // namespace TVC_Actuators