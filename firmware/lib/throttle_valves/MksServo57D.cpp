#include "MksServo57D.h"
#include "Arduino.h"
#include "math.h"

// Datasheet:
// https://github.com/makerbase-motor/MKS-SERVO42D-57D/blob/master/User%20Manual/V1.0.9/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.9.pdf
// Citations are indicated as [pg#].

// See also: https://github.com/whickmott/MKSServoCAN/blob/main/src/MKSServoCAN.cpp

void MksServo57D::begin() {
  // 1. General config: pins + mode (NORMAL, NO_ACK, LISTEN_ONLY)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);

  // 2. Timing config: pick the bitrate your bus uses
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // 3. Filter config: accept all messages (no filtering)
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    CommsSerial.println("Failed to install TWAI driver");
    return;
  }

  // Start driver
  if (twai_start() != ESP_OK) {
    CommsSerial.println("Failed to start TWAI driver");
    return;
  }

  CommsSerial.println("TWAI driver started");

  pinMode(GPIO_NUM_2, OUTPUT);
  digitalWrite(GPIO_NUM_2, HIGH);
  delay(100);
  digitalWrite(GPIO_NUM_2, LOW);
  delay(10);

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
