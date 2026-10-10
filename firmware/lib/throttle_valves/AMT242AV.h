#pragma once

#include "RS485.h"
#include <Arduino.h>

// Library for communicating with a single AMT242A-V absolute encoder using the RS485 bus class
class AMT242AV {
public:
  AMT242AV(RS485Device &device, uint8_t encoder_address);
  void begin();

  bool read_pos(float *out, int max_retries = 10);
  bool zero();
  bool reset();

private:
  RS485Device &device;
  uint8_t encoder_address_;

  bool _read_pos(uint16_t *);
};