#include "MksServo57D.h"
#include "Arduino.h"
#include "math.h"

// Datasheet:
// https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf
// Citations are indicated as [pg#].

// See also: https://github.com/whickmott/MKSServoCAN/blob/main/src/MKSServoCAN.cpp

FlexCAN_T4<CAN1, RX_SIZE_2, TX_SIZE_16> motorCAN;

void MksServo57D::begin() {
  std::array<uint8_t, 0> data{};
  send_frame(0x40, data);
}

// Command the speed and acceleration of the motor. See [72].
void MksServo57D::set_speed(int16_t speed, uint8_t acceleration) {
  std::array<uint8_t, 3> data{};
  const bool direction = speed > 0;

  // Max speed in open loop mode is 400 RPM. See [20].
  speed = constrain(abs(speed), 0, 400);

  // MSB is direction, lower 4 bits are the upper 4 bits of the 12 bit speed value.
  // Bits 4-6 intentionally left empty.
  data[0] = (direction << 7) | ((speed >> 4) & 0xF);
  data[1] = speed & 0xFF;
  data[2] = acceleration;

  send_frame(0xF6, data);
}
