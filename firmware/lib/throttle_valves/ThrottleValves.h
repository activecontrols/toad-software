#pragma once

#include "AMT242AV.h"
#include "MksServo57D.h"

class ThrottleValve {
public:
  ThrottleValve(uint16_t motor_can_id /*, HardwareSerial &enc_uart, unsigned int enc_DE, unsigned int enc_RE,
                unsigned int enc_ID */)
      : motor(motor_can_id) /*, encoder(enc_uart, enc_DE, enc_RE, enc_ID)*/ {};

  void begin();
  void stop();
  void set_position(float angle);

  // private:
  MksServo57D motor;
  // AMT242AV encoder;
};

namespace ThrottleValves {

bool begin();
void stop();
void set_angles_ox_fu(float ox_angle, float fu_angle);
// void handle_can_msg(const CAN_message_t &msg);

} // namespace ThrottleValves