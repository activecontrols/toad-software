#pragma once

#include "AMT242AV.h"
#include "MksServo57D.h"

enum valve_movement_mode_t {VALVE_MOVEMENT_MODE_POSITION, VALVE_MOVEMENT_MODE_CONSTANT_SPEED, VALVE_MOVEMENT_MODE_STOPPED};

class ThrottleValve {
public:
  ThrottleValve(uint16_t motor_can_id , HardwareSerial &enc_uart, unsigned int enc_DE, unsigned int enc_RE,
                unsigned int enc_ID )
      : motor(motor_can_id), encoder(enc_uart, enc_DE, enc_RE, enc_ID), target_angle(0), last_update_ms(0), K(50.0f), mode(VALVE_MOVEMENT_MODE_STOPPED) {};

  void begin();
  void update(bool log_csv); // update the control loop
  void stop();
  void set_position(float angle);


  // private:
  MksServo57D motor;
  AMT242AV encoder;
  float target_angle;
  uint32_t last_update_ms;
  float K;
  valve_movement_mode_t mode;
};

namespace ThrottleValves {

bool begin();
void stop();
void update(bool log_csv=false); // update all control loops
void set_angles_ox_fu(float ox_angle, float fu_angle);
// void handle_can_msg(const CAN_message_t &msg);

} // namespace ThrottleValves