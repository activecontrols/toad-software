#include "AMT242AV.h"

// max reading for a 12 bit encoder
#define MAX_READING ((1 << 12) - 1)

AMT242AV::AMT242AV(RS485Device &device, uint8_t encoder_address) : device(device), encoder_address_(encoder_address) {}

void AMT242AV::begin() {}

bool AMT242AV::_read_pos(uint16_t *out) {
  // Send address to get encoder position
  if (device.bus.write(encoder_address_) != 1)
    return false;

  // Read data
  uint8_t data[2];
  size_t n = device.read(data, sizeof(data), 150);

  // Fail if data is not the correct size
  if (n != sizeof(data))
    return false;

  // Define variables
  uint16_t transmission;
  uint16_t pos;
  uint8_t cs_transmission;
  uint8_t cs_real;

  // Build the transmission from the data
  transmission = ((uint16_t)data[1] << 8) | data[0];
  // Remove the top 2 checksum bits
  transmission &= 0b0011111111111111;
  // Remove the bottom 2 bits to obtain the 12-bit encoder position
  pos = transmission >> 2;

  // Isolate checksum from transmitted data
  cs_transmission = (data[1] >> 6) & 0b11;

  // Calculate real checksum
  cs_real = 0;

  for (int i = 0; i < 6; ++i) {
    cs_real ^= pos >> (i * 2);
  }

  // Odd parity
  cs_real = (~cs_real) & 0b11;

  if (cs_real != cs_transmission)
    return false;

  *out = pos;

  return true;
}

// read position as a fraction of total range of motion (outputs float between 0.0 and 1.0)
// (0.0 -> 0 degrees, ..., 1.0 -> 360 degrees)
// this encoder is 12 bit resolution which is approx. 0.1 degree resolution
bool AMT242AV::read_pos(float *out, int max_tries) {
  if (!device.beginTransaction())
    return false;

  uint16_t raw_pos;
  for (int i = 0; i < max_tries; ++i) {
    if (_read_pos(&raw_pos)) {
      *out = (float)raw_pos / MAX_READING;
      device.endTransaction();
      return true;
    }

    delayMicroseconds(150);
  }

  device.endTransaction();

  return false;
}

bool AMT242AV::zero() {
  if (!device.beginTransaction())
    return false;

  uint8_t message = encoder_address_ | 0x02;
  bool success = device.bus.write(message) == 1;

  device.endTransaction();

  return success;
}

bool AMT242AV::reset() {
  if (!device.beginTransaction())
    return false;

  uint8_t message = encoder_address_ | 0x03;
  bool success = device.bus.write(message) == 1;

  device.endTransaction();

  return success;
}