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

constexpr uint32_t ACTUATORS_CAN_BIT_RATE = 1000000;
bool tvc_debug_mode = false;

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
  uint32_t ASSUMED_ACTUATOR_TELEMETRY_INTERVAL_MS = 1000;
  uint32_t ACTUATOR_HANDSHAKE_TIMEOUT_MS = 2 * ASSUMED_ACTUATOR_TELEMETRY_INTERVAL_MS + 500;

  float actual_rate = CAN_TVC.begin(ACTUATORS_CAN_BIT_RATE);
  if (actual_rate <= 0.0f) {
    return false;
  }

  // both actuators broadcast telemetry on their own every ~ACTUATOR_HANDSHAKE_TIMEOUT_MS/2 ms. Just
  // listen until we've heard from both, or give up after the timeout.
  bool heard_pitch = false;
  bool heard_yaw = false;

  uint32_t start_time = millis();
  while (millis() - start_time < ACTUATOR_HANDSHAKE_TIMEOUT_MS) {
    while (CAN_TVC.available() > 0) {
      arduino::CanMsg msg = CAN_TVC.read();
      uint32_t id = msg.isStandardId() ? msg.getStandardId() : msg.getExtendedId();

      if (id == CAN_ID_TVC_PITCH) {
        heard_pitch = true;
      } else if (id == CAN_ID_TVC_YAW) {
        heard_yaw = true;
      }
    }

    if (heard_pitch && heard_yaw) {
      return true;
    }
  }

  return false;
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
      // Fixed-format decode for exactly what flight code acts on - no
      // letter-code dispatch, no unused fields.
      tvc_actuator_telemetry_t telem = parse_tvc_telemetry(msg.data);

      if (tvc_debug_mode) {
        // Full generic decode/print, bench debugging only - never feeds a
        // flight decision. Matches "KLMGHEFY", the factory-default txData.
        char decode_str[] = {'K', 'L', 'M', 'G', 'H', 'E', 'F', 'Y'};
        struct telem full_frame;
        parse_CAN_frame(msg.data, msg.data_length, decode_str, sizeof(decode_str), &full_frame);
      }
    }
    // CAN_ID_STEPPER_OX / CAN_ID_STEPPER_FU frames also arrive on this same
    // bus - dispatch to ThrottleValves here too once it reads feedback this way.
  }
}

} // namespace TVC_Actuators