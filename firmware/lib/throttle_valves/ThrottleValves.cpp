#include "ThrottleValves.h"
#include "ec_pins.h"
#include "toad_can_bus.h"

void ThrottleValve::begin() {
  motor.begin();
  encoder.begin();
}

void ThrottleValve::stop() {
  // TODO - do additional logic for enabling/disabling breaking, estop vs normal stop, etc.?
  motor.set_speed(0);
}

// TODO - if this turns the motor on, make sure we don't leave it on by mistake!
// could use heartbeat to solve
// TODO - PID controller logic
void ThrottleValve::set_position(float angle) {
  float K = 1; // TODO - set this constant

  float current_angle;
  // TODO - handle return value
  (void)encoder.read_pos(&current_angle);
  float target_speed = (angle - current_angle) * K;
  motor.set_speed(target_speed);
}

namespace ThrottleValves {

// TODO - set encoder ID?
ThrottleValve ox_valve(CAN_ID_STEPPER_OX, OX_ENC_RS485_BUS, PIN_OX_ENC_DE, PIN_OX_ENC_RE, 0);
ThrottleValve fu_valve(CAN_ID_STEPPER_FU, FU_ENC_RS485_BUS, PIN_FU_ENC_DE, PIN_FU_ENC_RE, 0);

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

// Set target position of both valves.
void set_angles_ox_fu(float ox_angle, float fu_angle) {
  ox_valve.set_position(ox_angle);
  fu_valve.set_position(fu_angle);
}

} // namespace ThrottleValves