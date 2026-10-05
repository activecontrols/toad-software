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
ThrottleValve ox_valve(CAN_ID_STEPPER_OX, ENC_OX_RS485_BUS, PIN_ENC_OX_SEL, 0);
ThrottleValve fu_valve(CAN_ID_STEPPER_FU, ENC_FU_RS485_BUS, PIN_ENC_FU_SEL, 0);

void set_position_cmd(const char *cmd) {
  float angle = 0.0f;
  char *motor = "";
  if (sscanf(cmd, "%s %f", &motor, &angle) != 2) {
    CommsSerial.println("Usage: motor_set_position <ox/fu> <angle>\n");
    return;
  }
  if (strcmp(motor, "ox") == 0) {
    ox_valve.set_position(angle);
  } else if (strcmp(motor, "fu") == 0) {
    fu_valve.set_position(angle);
  } else {
    CommsSerial.println("Motor must be 'ox' or 'fu'\n");
    return;
  }
}

bool begin() {
  ox_valve.begin();
  fu_valve.begin();

  // TODO - add rest of the commands
  CommandRouter::add(stop, "motor_stop");
  CommandRouter::add(set_position_cmd, "motor_set_position");

  return true; // TODO - check for telemetry from each motor
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