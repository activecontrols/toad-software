#include "ThrottleValves.h"
#include "ec_pins.h"
#include "toad_can_bus.h"

#define ENCODER_UPDATE_INTERVAL_MS 5
static uint32_t csv_time_start_ms = 0U;

// Wraps angle in degrees to [-180, 180] using fmod
static float wrapAngleDeg(float angle) {
  angle = std::fmod(angle + 180.0, 360.0);
  if (angle < 0.0) {
    angle += 360.0;
  }
  return angle - 180.0;
}

static float clamp(float a, float minimum, float maximum) {
  if (a < minimum)
    a = minimum;
  else if (a > maximum)
    a = maximum;

  return a;
}

void ThrottleValve::begin() {
  motor.begin();
  encoder.begin();
}

void ThrottleValve::stop() {
  // TODO - do additional logic for enabling/disabling breaking, estop vs normal stop, etc.?
  motor.set_speed(0);
  mode = VALVE_MOVEMENT_MODE_STOPPED;
}

// TODO - if this turns the motor on, make sure we don't leave it on by mistake!
// could use heartbeat to solve
// TODO - PID controller logic
void ThrottleValve::set_position(float angle) {
  target_angle = angle;
  mode = VALVE_MOVEMENT_MODE_POSITION;
}

void ThrottleValve::set_speed(int16_t speed) {
  motor.set_speed(speed);
  mode = VALVE_MOVEMENT_MODE_CONSTANT_SPEED;
}

bool ThrottleValve::read_position(float *pos) {
  return encoder.read_pos(pos);
}

bool ThrottleValve::zero_encoder() {
  return encoder.zero();
}

void ThrottleValve::update(bool log_csv) {
  if (log_csv)
    log_csv_sticky = true;

  // no control system to update in these two cases
  if (mode == VALVE_MOVEMENT_MODE_STOPPED)
    return;
  if (mode == VALVE_MOVEMENT_MODE_CONSTANT_SPEED)
    return;

  if (millis() - last_update_ms < ENCODER_UPDATE_INTERVAL_MS)
    return;

  float current_pos = 0.0f;

  if (!encoder.read_pos(&current_pos)) {
    last_update_ms = millis();
    return; // error occurred while trying to read encoder position
  }

  float current_angle = 360.0f - current_pos * 360.0f; // convert position to deg

  float error = wrapAngleDeg(target_angle - current_angle);
  float target_speed = error * K; // proportional controller

  // clamp target speed to safe values
  target_speed = clamp(target_speed, -100.0f, 100.0f);

  // prevent weird jitter
  if (std::fabs(error) < 0.4f)
    target_speed = 0;

  if (log_csv_sticky) {
    float now_time_s = (millis() - csv_time_start_ms) * 1e-3;
    log_csv_sticky = false;
    CommsSerial.printf("%.3f,%.2f,%.2f,%.2f,%.2f\n", now_time_s, error, target_speed, current_angle, target_angle);
  }

  motor.set_speed(target_speed, 250U);
  // motor.set_speed(target_speed, 50U);

  last_update_ms = millis();
  return;
}

namespace ThrottleValves {

ThrottleValve ox_valve(CAN_ID_STEPPER_OX, RS485s::enc_ox, 0x54);
ThrottleValve fu_valve(CAN_ID_STEPPER_FU, RS485s::enc_fu, 0x54);

// make those templated functions

void set_ox_position_cmd(const char *cmd) {
  float angle = 0.0f;
  if (sscanf(cmd, "%f", &angle) != 1) {
    CommsSerial.println("Usage: motor_set_position <angle>\n");
    return;
  }

  ox_valve.set_position(angle);
  return;
}

void set_ox_speed_cmd(const char *cmd) {
  int16_t spd;
  if (sscanf(cmd, "%hd", &spd) != 1) {
    CommsSerial.println("Usage: set_speed rpm");
    return;
  }
  ox_valve.set_speed(spd);
}

void get_ox_position_cmd(const char *cmd) {
  float pos = 0.0f;

  if (!ox_valve.read_position(&pos)) {
    CommsSerial.println("Failed to read encoder position\n");
  } else {
    CommsSerial.printf("Position: %.2f deg\n", pos * 360.0f);
  }
}

void stop_ox_valve_cmd(const char *cmd) {
  (void)cmd;

  ox_valve.stop();

  return;
}

void zero_ox_encder_cmd(const char *cmd) {
  (void)cmd;

  ox_valve.zero_encoder();

  return;
}

// This probably won't be used for static test / actual vehicle because control code should go somewhere else
void motor_follow_profile_cmd(const char *cmd) {
  csv_time_start_ms = millis();
  CommsSerial.print("Time (s),Error (deg),Target Speed (rpm),Current Angle (deg),Target Angle (deg)\n");

  bool do_log = false; // wait for command from the python script before logging
  for (;;) {
    char cmd = ' ';
    while (CommsSerial.available()) {
      cmd = CommsSerial.read();

      // "goto" command and "quit" command
      if (cmd == 'g' || cmd == 'q') {
        break;
      }
    }

    if (cmd == 'q')
      break;

    if (cmd == 'g') {
      String arg = CommsSerial.readStringUntil('\n');

      float target_pos = arg.toFloat();

      ox_valve.set_position(target_pos);
      do_log = true; // log the next control loop update so the python script has new data to work with
    }

    update(do_log);
    do_log = false;
  }
}

// TODO - don't just return true here!
bool begin() {
  ox_valve.begin();
  fu_valve.begin();

  CommandRouter::add(set_ox_position_cmd, "set_ox_position");
  CommandRouter::add(set_ox_speed_cmd, "set_ox_speed");
  CommandRouter::add(get_ox_position_cmd, "get_ox_position");
  CommandRouter::add(stop_ox_valve_cmd, "stop_ox_valve");
  CommandRouter::add(zero_ox_encder_cmd, "zero_ox_encoder");

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

void update(bool log_csv) {
  ox_valve.update(log_csv);
  fu_valve.update(log_csv);
}

} // namespace ThrottleValves