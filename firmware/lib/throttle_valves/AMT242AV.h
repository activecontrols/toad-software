#pragma once

#include <Arduino.h>

// library for communicating with an AMT242A-V absolute encoder (over a uart interface -> MAX485 module -> absolute encoder)
class AMT242AV {
public:
  AMT242AV(HardwareSerial &uart, unsigned int DE, unsigned int RE, uint8_t ID);
  void begin();

  bool read_pos(float *out, int max_retries = 10);
  void zero();
  void reset();

private:
  HardwareSerial &uart;
  unsigned int DE;
  unsigned int RE;
  uint8_t ID;

  bool wait_for_avail(unsigned long long);
  bool _read_pos(uint16_t *);
};