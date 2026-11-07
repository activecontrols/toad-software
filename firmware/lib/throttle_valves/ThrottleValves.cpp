#include "ThrottleValves.h"
#include "ec_pins.h"
#include "toad_can_bus.h"

#define ENCODER_UPDATE_INTERVAL_MS 5
static uint32_t csv_time_start_ms = 0U;

// Encoder RS-485 address, identical for fu and ox because they're on different lines
constexpr int ENCODER_ID = 0x54;

// Gear ratio on the valve, 10:1.
constexpr int16_t GEAR_RATIO = 10;

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

void ThrottleValve::set_speed(int16_t speed, int16_t acceleration) {
  motor.set_speed(speed, acceleration);
  mode = VALVE_MOVEMENT_MODE_CONSTANT_SPEED;
}

bool ThrottleValve::read_position(float *pos) {
  float new_pos = 0.0f;
  if (encoder.read_pos(&new_pos) == false) {
    return false;
  }

  *pos = 360.0f - new_pos * 360.0f;
  return true;
}

bool ThrottleValve::zero_encoder() {
  return encoder.zero();
}

// Rotates the motor until it stops, then zeroes the encoder.
// Blocks the control loop, so should only run before the control loop starts.
// Another concern: This can't tell if the motor has reached the hard stop or if it's broken down in some way.
bool ThrottleValve::zero_valve() {
  // Actual target output RPM (gearbox considered below)
  constexpr int16_t zero_angular_velocity = 10;
  // How many degrees must the motor move to be considered "started"?
  constexpr int motor_started_moving_threshold = 1;
  // At what percentage of zero_angular_velocity is the motor considered "stopped"?
  constexpr int stopped_velocity_percent_threshold = 10;
  // How long after motor startup do we start listening for motor stops?
  constexpr uint32_t motor_startup_us = 50000;
  // Timeout
  constexpr uint32_t timeout_us = 2000000;
  // How long do we want the motor to remain still before zeroing?
  constexpr int stopped_counter_limit = 5;

  constexpr float stopped_velocity_threshold = zero_angular_velocity * stopped_velocity_percent_threshold / 100.0f;

  // Get initial previous angle
  float logging_angle = 0U;
  if (!read_position(&logging_angle)) {
    stop();
    return false;
  }
  CommsSerial.printf("Starting Valve Zero, About to Move Backward: %f\n", logging_angle);

  // Move backwards slightly
  set_speed(-zero_angular_velocity * GEAR_RATIO);
  delay(100);
  stop();

  uint32_t start_us = micros();
  uint32_t prev_us = start_us;
  uint32_t current_us = start_us;
  float prev_angle = 0U;
  // Has the motor moved from its initial position?
  bool moved = false;
  // How long has the motor remained still?
  int stopped_counter = 0;

  int print_position_clock = 0;

  // Start moving forward
  set_speed(zero_angular_velocity * GEAR_RATIO);

  // Get initial previous angle
  if (!read_position(&prev_angle)) {
    stop();
    return false;
  }

  float start_angle = prev_angle;
  float current_angle = prev_angle;

  CommsSerial.printf("Starting Valve Zero Loop, About to move Forward: %f\n", current_angle);

  // Returned boolean
  bool success = false;
  // Loop
  while (current_us - start_us < timeout_us) {
    // Get current encoder position
    if (!read_position(&current_angle)) {
      break;
    }

    current_us = micros();

    // Prevent division by zero if time hasn't changed
    if (current_us == prev_us) {
      delay(2);
      continue;
    }

    // Check if motor has moved since startup.
    // Used to prevent stop detection if the motor never moves in the first place.
    if (fabsf(wrapAngleDeg(current_angle - start_angle)) > motor_started_moving_threshold) {
      moved = true;
    }

    // Calculate current velocity
    float angle_difference = fabsf(wrapAngleDeg(current_angle - prev_angle));               // Degrees
    float time_difference = (current_us - prev_us) / 1000000.0f;                            // Seconds
    float current_angular_velocity = (angle_difference / time_difference) * 60.0f / 360.0f; // RPM

    // If the motor has moved before, is stopped, and motor_startup_ms has passed, increment stopped_counter
    bool is_stopped = current_angular_velocity <= stopped_velocity_threshold;
    if (moved && is_stopped && ((current_us - start_us) > motor_startup_us)) {
      stopped_counter++;
    } else {
      stopped_counter = 0;
    }

    // Zero the encoder if the motor has been stopped long enough
    if (stopped_counter >= stopped_counter_limit) {
      stop();
      delay(2);
      success = zero_encoder();
      break;
    }

    // Cancel when comms serial is available
    if (CommsSerial.available()) {
      break;
    }

    // Update velocity calc values
    prev_angle = current_angle;
    prev_us = current_us;
    print_position_clock++;

    if (print_position_clock > 50) {
      print_position_clock = 0;
      CommsSerial.printf("Current Zero Position: %f\n", current_angle);
    }

    delay(10);
  }

  // Stop motor and return
  stop();
  CommsSerial.printf("Ending Valve Zero: %f\n", current_angle);
  return success;
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

  // Get current encoder position
  float current_angle = 0.0f;
  if (!read_position(&current_angle)) {
    last_update_ms = millis();
    return; // error occurred while trying to read encoder position
  }

  float error = wrapAngleDeg(target_angle - current_angle);
  float target_speed = error * K; // proportional controller

  // clamp target speed to safe values
  target_speed = clamp(target_speed, -300.0f, 300.0f);

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

ThrottleValve ox_valve(CAN_ID_STEPPER_OX, RS485s::enc_ox, ENCODER_ID);
ThrottleValve fu_valve(CAN_ID_STEPPER_FU, RS485s::enc_fu, ENCODER_ID);

// Templated position command function
template <ThrottleValve &valve, bool is_ox> void set_position_cmd(const char *cmd) {
  float angle = 0.0f;
  if (sscanf(cmd, "%f", &angle) != 1) {
    if constexpr (is_ox) {
      CommsSerial.println("Usage: set_ox_position <angle>");
    } else {
      CommsSerial.println("Usage: set_fu_position <angle>");
    }
    return;
  }
  valve.set_position(angle);
}

// Templated speed command function
template <ThrottleValve &valve, bool is_ox> void set_speed_cmd(const char *cmd) {
  int16_t spd;
  if (sscanf(cmd, "%hd", &spd) != 1) {
    if constexpr (is_ox) {
      CommsSerial.println("Usage: set_ox_speed <rpm>");
    } else {
      CommsSerial.println("Usage: set_fu_speed <rpm>");
    }
    return;
  }
  valve.set_speed(spd);
}

// Templated get position command function
template <ThrottleValve &valve> void get_position_cmd() {
  float pos = 0.0f;

  if (!valve.read_position(&pos)) {
    CommsSerial.println("Failed to read encoder position\n");
  } else {
    CommsSerial.printf("Position: %.2f deg\n", pos);
  }
}

// Templated stop command function
template <ThrottleValve &valve> void stop_valve_cmd() {
  valve.stop();
}

// Templated zero encoder command function
template <ThrottleValve &valve> void zero_encoder_cmd() {
  valve.zero_encoder();
}

// Templated zero valve command function
template <ThrottleValve &valve> void zero_valve_cmd() {
  if (valve.zero_valve()) {
    CommsSerial.println("Valve zeroed successfully");
  } else {
    CommsSerial.println("Valve zeroing failed");
  }
}

// This probably won't be used for static test / actual vehicle because control code should go somewhere else
// This should probably be removed
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

  CommandRouter::add(set_position_cmd<ox_valve, true>, "set_ox_position");
  CommandRouter::add(set_speed_cmd<ox_valve, true>, "set_ox_speed");
  CommandRouter::add(get_position_cmd<ox_valve>, "get_ox_position");
  CommandRouter::add(stop_valve_cmd<ox_valve>, "stop_ox_valve");
  CommandRouter::add(zero_encoder_cmd<ox_valve>, "zero_ox_encoder");
  CommandRouter::add(zero_valve_cmd<ox_valve>, "zero_ox_valve");

  CommandRouter::add(set_position_cmd<fu_valve, false>, "set_fu_position");
  CommandRouter::add(set_speed_cmd<fu_valve, false>, "set_fu_speed");
  CommandRouter::add(get_position_cmd<fu_valve>, "get_fu_position");
  CommandRouter::add(stop_valve_cmd<fu_valve>, "stop_fu_valve");
  CommandRouter::add(zero_encoder_cmd<fu_valve>, "zero_fu_encoder");
  CommandRouter::add(zero_valve_cmd<fu_valve>, "zero_fu_valve");

  CommandRouter::add(motor_follow_profile_cmd, "motor_follow_profile");

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