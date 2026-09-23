#include "TVC_Actuators.h"
#include "GimbalKinematics.h"
#include "UltramotionActuator.hpp"
#include "ec_pins.h"
#include "fdcan_toad.h"
#include "toad_can_bus.h"

// One CAN object here, not two - CAN_ID_TVC_PITCH and CAN_ID_TVC_YAW are two
// message IDs on the SAME physical "Actuators CAN Bus" (PIN_CAN_TVC_RX/TX),
// not two separate buses. Note the constructor takes (tx_pin, rx_pin), in
// that order - matches CAN::CAN(uint32_t _tx_pin, uint32_t _rx_pin) in
// fdcan_toad.cpp.

namespace TVC_Actuators {

CAN actuators_can(PIN_CAN_TVC_TX, PIN_CAN_TVC_RX);

// TODO - PLACEHOLDER. 500 kbit/s is a common CAN default, not a confirmed
// value - needs to match whatever the Ultramotion actuators are configured
constexpr uint32_t ACTUATORS_CAN_BIT_RATE = 500000;

// TODO - PLACEHOLDER scaling. Assumes a straight linear map from physical
// actuator length (mm) to the actuator's raw target_pos range (0-65535).
// Needs the actuator's real stroke length and pMin/pMax from its
// datasheet/CONFIG.TXT before this is trustworthy.
uint16_t length_to_target_pos(float length_mm) {
  constexpr float MIN_LENGTH_MM = 0.0f;   // TODO
  constexpr float MAX_LENGTH_MM = 100.0f; // TODO
  constexpr uint16_t POS_MAX = 65535;

  float clamped = constrain(length_mm, MIN_LENGTH_MM, MAX_LENGTH_MM);
  return (uint16_t)((clamped - MIN_LENGTH_MM) / (MAX_LENGTH_MM - MIN_LENGTH_MM) * POS_MAX);
}

bool begin() {
  float actual_rate = actuators_can.begin(ACTUATORS_CAN_BIT_RATE);
  reset_status_state(); // from UltramotionActuator.hpp - clears "what's new" status tracking
  return actual_rate > 0.0f;
}

void set_angles_pitch_yaw(float pitch, float yaw) {
  float pitch_len, yaw_len;
  calc_actuator_lengths(pitch, yaw, &pitch_len, &yaw_len); // already implemented

  send_target_pos(actuators_can, CAN_ID_TVC_PITCH, length_to_target_pos(pitch_len));
  send_target_pos(actuators_can, CAN_ID_TVC_YAW, length_to_target_pos(yaw_len));
}

// Call every flight_loop() iteration - nothing currently does. Drains
// whatever arrived in the RX FIFO since the last call and decodes anything
// addressed to the TVC actuators.
void poll() {
  while (actuators_can.available() > 0) {
    arduino::CanMsg msg = actuators_can.read();
    uint32_t id = msg.isStandardId() ? msg.getStandardId() : msg.getExtendedId();

    if (id == CAN_ID_TVC_PITCH || id == CAN_ID_TVC_YAW) {
      // TODO - confirm this 6-byte [status_word][position] decode_str
      // against the actuator's actual configured telemetry layout - still
      // an open team decision, not a confirmed spec.
      char decode_str[] = {'A', 'B', 'C', 'D', 'G', 'H'};
      telem frame;
      parse_CAN_frame(msg.data, msg.data_length, decode_str, sizeof(decode_str), &frame);

      // TODO - act on frame.status_word here, e.g.:
      //   constexpr uint32_t FOLLOWING_ERROR_BIT = 1u << 11;
      //   if (frame.status_word & FOLLOWING_ERROR_BIT) { kill_flag = true; }
      (void)frame;
    }
    // CAN_ID_STEPPER_OX / CAN_ID_STEPPER_FU frames also arrive on this same
    // bus - dispatch to ThrottleValves here too once it reads feedback this way.
  }
}

} // namespace TVC_Actuators