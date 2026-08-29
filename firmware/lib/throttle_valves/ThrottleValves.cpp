#include "ThrottleValves.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "ec_pins.h"
#include "toad_can_bus.h"
#include <cmath>

#ifdef DEVICE_ESP32
#include "driver/uart.h"
#endif

#define ENCODER_UPDATE_INTERVAL_MS 5



// Wraps angle in degrees to [-180, 180] using fmod
static float wrapAngleDeg(float angle) {
  angle = std::fmod(angle + 180.0, 360.0);
  if (angle < 0.0) {
      angle += 360.0;
  }
  return angle - 180.0;
}


static float clamp(float a, float minimum, float maximum)
{
  if (a < minimum) a = minimum;
  else if (a > maximum) a = maximum;

  return a;
}

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
  target_angle = angle;
  mode = VALVE_MOVEMENT_MODE_POSITION;
}

void ThrottleValve::update(void)
{
  // no control system to update in these two cases
  if (mode == VALVE_MOVEMENT_MODE_STOPPED) return;
  if (mode == VALVE_MOVEMENT_MODE_CONSTANT_SPEED) return;

  if (millis() - last_update_ms < ENCODER_UPDATE_INTERVAL_MS) return;

  float current_pos = 0.0f;

  if (!encoder.read_pos(&current_pos))
  {
    last_update_ms = millis();
    return; // error occurred while trying to read encoder position
  }
  
  float current_angle = -current_pos * 360.0f; // convert position to deg

  float error = wrapAngleDeg(target_angle - current_angle);
  float target_speed = error * K; // proportional controller

  // clamp target speed to safe values
  target_speed = clamp(target_speed, -200.0f, 200.0f);


  // prevent weird jitter
  if (std::fabs(error) < 0.4f) target_speed = 0;
  else
  {
    CommsSerial.printf("Error: %.1f\tTarget Speed: %.1f\tAngle: %.1f\n", error, target_speed, current_angle);
  }

  motor.set_speed(target_speed, 250U);

  last_update_ms = millis();
  return;
}

namespace ThrottleValves {


// TODO: change for EC; this is for esp32 testing (RE/DE on D5)

// note: on esp32, AMT242AV.cpp hardcodes uart 2 as the serial output (see AMT242AV::_read_pos())
HardwareSerial rs485_test_ser(2);

// using d14 as a placeholder
ThrottleValve ox_valve(CAN_ID_STEPPER_OX, rs485_test_ser, 14, 14, 0x54);
// ThrottleValve fu_valve(CAN_ID_STEPPER_FU /*, FU_ENC_RS485_BUS, PIN_FU_ENC_DE, PIN_FU_ENC_RE, 0*/);

void set_position_cmd(const char* cmd)\
{
  float angle = 0.0f;
  if (sscanf(cmd, "%f", &angle) == 0)
  {
    CommsSerial.println("Usage: motor_set_position <angle>\n");
    return;
  }

  ox_valve.set_position(angle);
  return;
}

void set_speed_cmd(const char *cmd) {
  int16_t spd;
  if (sscanf(cmd, "%hd", &spd) != 1) {
    CommsSerial.println("Usage: set_speed rpm");
    return;
  }
  ox_valve.motor.set_speed(spd);
  ox_valve.mode = VALVE_MOVEMENT_MODE_CONSTANT_SPEED;
}

void get_position_cmd(const char* cmd)
{
  float pos = 0.0f;

  if (!ox_valve.encoder.read_pos(&pos))
  {
    CommsSerial.println("Failed to read encoder position\n");
  }
  else
  {
    CommsSerial.printf("Position: %.2f deg\n", pos * 360.0f);
  }
}

// TODO - don't just return true here!
bool begin() {
  rs485_test_ser.begin(2000000);

  #ifdef DEVICE_ESP32
  
  // this assumes we are using uart 2
  uart_set_pin(2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 5, UART_PIN_NO_CHANGE); // Add Pin 5 as RTS - the uart controller automatically toggles RE/DE
  uart_set_mode(2, UART_MODE_RS485_HALF_DUPLEX);   // Enable hardware toggling
  #endif

  ox_valve.begin();

  // fu_valve.begin();

  CommandRouter::add(set_speed_cmd, "motor_speed");
  CommandRouter::add(stop, "motor_stop");
  CommandRouter::add(get_position_cmd, "motor_get_position");
  CommandRouter::add(set_position_cmd, "motor_set_position");

  return true;
}

// Stop both throttle valve motors.
void stop() {
  ox_valve.stop();
  // fu_valve.stop();
}

// Set target position of both valves.
void set_angles_ox_fu(float ox_angle, float fu_angle) {
  ox_valve.set_position(ox_angle);
  // fu_valve.set_position(fu_angle);
}

// // TODO - deal with status messages across different acutators
// void handle_can_msg(const CAN_message_t &msg) {
//   CommsSerial.printf("New can msg id=%d len=%d\n", msg.id, msg.len);
//   for (size_t i = 0; i < msg.len; i++) {
//     CommsSerial.printf("%d\n", msg.buf[i]);
//   }
// }

void update()
{
  // update all valves
  ox_valve.update(); 
}

} // namespace ThrottleValves