#pragma once

#include "MksServo57D.h"

class ThrottleValve {
public:
  ThrottleValve(uint16_t motor_can_id) : motor(motor_can_id) {};

  void begin();
  void stop();
  void set_position(float angle);

private:
  MksServo57D motor;
};

namespace ThrottleValves {

bool begin();
void stop();
void set_angles_ox_fu(float ox_angle, float fu_angle);

} // namespace ThrottleValves