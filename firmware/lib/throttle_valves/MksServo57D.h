#pragma once

#include <stdint.h>

class MksServo57D {
public:
  MksServo57D(uint16_t can_id) : can_id(can_id) {};
  void begin();
  void set_speed();

private:
  uint16_t can_id;
};