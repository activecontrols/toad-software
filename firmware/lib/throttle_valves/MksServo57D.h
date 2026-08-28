#pragma once

#include <array>
#include <stdint.h>
using std::size_t;

class MksServo57D {
public:
  MksServo57D(uint16_t can_id) : can_id(can_id) {};
  void begin();
  // TODO - set default acceleration
  // TODO - acceleration does not handle switching direction - might want to implement that ourselves.
  void set_speed(int16_t speed, uint8_t acceleration = 32);

private:
  // The CAN protocol for the MKS Servo 57D uses Big Endian. Convert an numeric value into a correctly sized array of
  // its bytes in Big Endian order. Call as to_be_bytes<uint#_t>() to specify the type explicitly.
  template <typename T> std::array<uint8_t, sizeof(T)> to_be_bytes(T value) {
    std::array<uint8_t, sizeof(T)> bytes;

    for (size_t i = 0; i < sizeof(T); i++) {
      bytes[i] = static_cast<uint8_t>(value >> (8 * (sizeof(T) - 1 - i)));
    }

    // Using std::array allows returning the array by value, which is convenient for passing into send_frame.
    // As the max frame size is 8 bytes, this is a reasonable size to return, a the compiler can easily optimize out the
    // copies here as no heap allocation is used.
    return bytes;
  }

  // Build and send a MKS CAN frame from the provided array of bytes. CAN frames start with the command, followed by
  // data (typically a single numeric value in Big Endian order), followed by a CRC, which is simply the sum of all
  // previous bytes, including the message CAN id, restricted to 1 byte. [25]
  // TODO - consider making uint8_t cmd an enum or similar
  template <size_t N> void send_frame(uint8_t cmd, const std::array<uint8_t, N> &data) {
    constexpr size_t frame_len = N + 2;
    static_assert(frame_len <= 8, "CAN 2.0 has a max frame length of 8 bytes.");
    uint8_t frame[frame_len];

    frame[0] = cmd;
    for (size_t i = 0; i < N; i++) {
      frame[i + 1] = data[i];
    }

    uint8_t crc = can_id;
    for (size_t i = 0; i < N + 1; i++) {
      crc += frame[i];
    }
    frame[N + 1] = crc;

    // TODO - transmit the frame
  }

  uint16_t can_id;
};