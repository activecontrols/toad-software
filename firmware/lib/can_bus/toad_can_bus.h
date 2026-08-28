#pragma once
#include <optional>
#include <stdint.h>
#include <string.h>

/// BUS TOPOLOGY
// The primary vehicle CAN bus runs from the flight
// controller to the engine controller and includes
// the power management board. This is a CAN-FD bus.

// The engine control CAN bus runs from the engine
// controller to the TVC actuators and the stepper
// drivers. This is a CAN 2.0 bus.

// The GSE CAN bus runs from the GSE, over the QD
// arm and splits to run to each programmer board.
// This is a CAN-FD bus.

/// CAN IDs
// Each CAN device has an 11 bit ID.

// COTS devices (ID range 0x00X)
constexpr uint16_t CAN_ID_TVC_PITCH = 0x001;
constexpr uint16_t CAN_ID_TVC_YAW = 0x002;
constexpr uint16_t CAN_ID_STEPPER_OX = 0x001;
constexpr uint16_t CAN_ID_STEPPER_FU = 0x001;

// Custom boards (ID range 0x01X)
constexpr uint16_t CAN_ID_FLIGHT_CONTROLLER = 0x011;
constexpr uint16_t CAN_ID_ENGINE_CONTROLLER = 0x012;
constexpr uint16_t CAN_ID_FLIGHT_PROG = 0x013;
constexpr uint16_t CAN_ID_ENGINE_PROG = 0x014;
constexpr uint16_t CAN_ID_GSE = 0x015;
constexpr uint16_t CAN_ID_POWER_BOARD = 0x016;

// /// CAN Messages
// // Each custom CAN message starts with a 1 byte command ID.
// // Messages can be 1 - 64 bytes.

// // Heartbeat / Status Messages (ID range 0b0000_XXXX)
// struct can_msg_heartbeat_t {
//   uint8_t cmd_id = 0x00;
// };

// // Sent as an error reply when device receives a command with an invalid ID,
// // or that command is not supported on this device.
// struct can_msg_invalid_cmd_t {
//   uint8_t cmd_id = 0x01;
//   uint8_t rcv_cmd_id; // The command that was received.
// };

// // Sent as an error reply when device receives a command with a payload that did not match the expected length.
// struct can_msg_incorrect_len_t {
//   uint8_t cmd_id = 0x02;
//   uint8_t rcv_cmd_id;   // The command that was received.
//   uint8_t len;          // Payload length of received command.
//   uint8_t expected_len; // Expected payload length of command with received command's id.
// };

// // Sent as an error reply when device receives a command that is not valid in its current state machine state.
// struct can_msg_unexpected_state_t {
//   uint8_t cmd_id = 0x03;
//   uint8_t rcv_cmd_id;     // The command that was received.
//   uint8_t state;          // State machine state when cmd was received.
//   uint8_t expected_state; // State machine state required for cmd.
// };

// // Telemetry Messages (ID range 0b0001_XXXX)
// struct can_msg_fc_telemetry {
//   uint8_t cmd_id = 0x10;
// };

// struct can_msg_ec_telemetry {
//   uint8_t cmd_id = 0x11;
// };

// // Flight Commands (ID range 0b0010_XXXX)

// // Flash Control (ID range 0b0011_XXXX)
// struct can_msg_reset_controller_t {
//   uint8_t cmd_id = 0x30;
// };

// struct can_msg_enter_bootloader_t {
//   uint8_t cmd_id = 0x31;
// };

// struct can_msg_erase_flash_t {
//   uint8_t cmd_id = 0x32;
// };

// struct can_msg_select_page_t {
//   uint8_t cmd_id = 0x33;
//   uint16_t page_addr;
// };

// struct can_msg_mem_packet_t {
//   uint8_t cmd_id = 0x34;
//   uint8_t chunk_addr;
//   uint16_t page_addr;
//   uint8_t flash_bytes[32];
// };

// struct can_msg_request_mem_packet_t {
//   uint8_t cmd_id = 0x35;
//   uint8_t chunk_addr;
//   uint16_t page_addr;
// };

// struct can_msg_write_flash_t {
//   uint8_t cmd_id = 0x36;
// };

// // RESERVED for Extended Command Format (ID range 0b0111_XXXX)

// // Custom Command over CAN (ID range 0b1XXY_YYYY)

// /// CAN Helper Functions

// template <typename state_t> class CAN_Msg_Decoder {

// public:
//   // Helper for decoding a raw buffer into a can msg with error handling.
//   CAN_Msg_Decoder(const uint8_t *raw_bytes, size_t len, state_t state)
//       : decoded(false), raw_bytes(raw_bytes), len(len), state(state) {};

//   // Decode a raw buffer into a CAN message of the specified type and verify the state machine state.
//   // If the command has already been decoded, does nothing and returns std::nullopt.
//   // If the command id does not match, does nothing and returns std::nullopt.
//   // If the payload length is incorrect, sends can_msg_incorrect_len and returns std::nullopt.
//   // If the state machine does not match the expected state, sends can_msg_unexpected_state and returns std::nullopt.
//   // Usage: `if (const auto msg = raw_msg.decode<can_msg_t>()) {`
//   template <typename msg_t> std::optional<msg_t> decode_and_enforce_state(state_t expected) {
//     // If the message was already decoded, exit early and
//     // return std::nullopt so this can be used in an if/else chain.
//     if (decoded) {
//       return std::nullopt;
//     }

//     msg_t msg;

//     // The cmd_id is always the first byte of the message. If the command id doesn't match,
//     // then return std::nullopt so this can be used in an if/else chain.
//     const uint8_t cmd = raw_bytes[0];
//     if (cmd != msg.cmd_id) {
//       return std::nullopt;
//     }

//     // The command id matched, so mark this message as decoded.
//     decoded = true;

//     if (len != sizeof(msg)) {
//       can_msg_incorrect_len_t error_msg;
//       // TODO - send
//       return std::nullopt;
//     }

//     if (state != expected) {
//       can_msg_unexpected_state_t error_msg;
//       error_msg.rcv_cmd_id = msg.cmd_id;
//       error_msg.expected_state = expected;
//       error_msg.state = state;
//       // TODO - send
//       return std::nullopt;
//     }

//     memcpy(&msg, raw_bytes, sizeof(msg));
//     return msg;
//   }

//   // Decode a raw buffer into a CAN message of the specified type.
//   // If the command has already been decoded, does nothing and returns std::nullopt.
//   // If the command id does not match, does nothing and returns std::nullopt.
//   // If the payload length is incorrect, sends can_msg_incorrect_len and returns std::nullopt.
//   // Usage: `if (const auto msg = raw_msg.decode<can_msg_t>()) {`
//   template <typename msg_t> std::optional<msg_t> decode() {
//     return decode_and_enforce_state<msg_t>(state);
//   }

//   // After a set of `decode` and `decode_and_enforce_state`, this function sends
//   // can_msg_invalid_cmd if the command was never decoded.
//   void send_error_if_not_decoded() {
//     if (!decoded) {
//       can_msg_invalid_cmd_t error_msg;
//       error_msg.rcv_cmd_id = raw_bytes[0];
//       // TODO - send CAN_MSG_INVALID_CMD
//     }
//   }

// private:
//   bool decoded;
//   const uint8_t *raw_bytes;
//   const size_t len;
//   const state_t state;
// };
