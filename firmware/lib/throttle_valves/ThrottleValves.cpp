#include "ThrottleValves.h"
#include "toad_can_bus.h"

void ThrottleValve::begin() {
  motor.begin();
}

void ThrottleValve::stop() {
  // TODO - do additional logic for enabling/disabling breaking, estop vs normal stop, etc.?
  motor.set_speed(/* 0 */);
}

// TODO - if this turns the motor on, make sure we don't leave it on by mistake!
// could use heartbeat to solve
// TODO - control loop logic
void ThrottleValve::set_position(float angle) {
  // current_angle = encoder.get_pos();
  // target_speed = (angle - current_angle) * K;
  motor.set_speed();
}

namespace ThrottleValves {

ThrottleValve ox_valve(CAN_ID_STEPPER_OX);
ThrottleValve fu_valve(CAN_ID_STEPPER_FU);

// TODO - don't just return true here!
bool begin() {
  ox_valve.begin();
  fu_valve.begin();

  return true;
}

// Stop both throttle valve motors.
void stop() {
  ox_valve.stop();
  fu_valve.stop();
}

// TODO - this function
void set_angles_ox_fu(float ox_angle, float fu_angle) {}

} // namespace ThrottleValves