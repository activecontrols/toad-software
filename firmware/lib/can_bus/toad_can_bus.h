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
constexpr uint16_t CAN_ID_STEPPER_OX = 0x003;
constexpr uint16_t CAN_ID_STEPPER_FU = 0x004;

// Custom boards (ID range 0x01X)
constexpr uint16_t CAN_ID_FLIGHT_CONTROLLER = 0x011;
constexpr uint16_t CAN_ID_ENGINE_CONTROLLER = 0x012;
constexpr uint16_t CAN_ID_FLIGHT_PROG = 0x013;
constexpr uint16_t CAN_ID_ENGINE_PROG = 0x014;
constexpr uint16_t CAN_ID_GSE = 0x015;
constexpr uint16_t CAN_ID_POWER_BOARD = 0x016;

/// CAN Messages
// Each custom CAN message starts with a 1 byte command ID.
// Messages can be 1 - 64 bytes.

// Heartbeat / Status Messages (ID range 0b0000_XXXX)
struct can_msg_heartbeat_t {
  static constexpr uint8_t cmd_id = 0x00;
};

// Sent as an error reply when device receives a command with an invalid ID,
// or that command is not supported on this device.
struct can_msg_invalid_cmd_t {
  static constexpr uint8_t cmd_id = 0x01;
  uint8_t rcv_cmd_id; // The command that was received.
};

// Sent as an error reply when device receives a command with a payload that did not match the expected length.
struct can_msg_incorrect_len_t {
  static constexpr uint8_t cmd_id = 0x02;
  uint8_t rcv_cmd_id;   // The command that was received.
  uint8_t len;          // Payload length of received command.
  uint8_t expected_len; // Expected payload length of command with received command's id.
};

// Sent as an error reply when device receives a command that is not valid in its current state machine state.
struct can_msg_unexpected_state_t {
  static constexpr uint8_t cmd_id = 0x03;
  uint8_t rcv_cmd_id;     // The command that was received.
  uint8_t state;          // State machine state when cmd was received.
  uint8_t expected_state; // State machine state required for cmd.
};

// Telemetry Messages (ID range 0b0001_XXXX)
struct can_msg_fc_telemetry {
  static constexpr uint8_t cmd_id = 0x10;
};

struct can_msg_ec_telemetry {
  static constexpr uint8_t cmd_id = 0x11;
};

// Flight Commands (ID range 0b0010_XXXX)

// Flash Control (ID range 0b0011_XXXX)
struct can_msg_reset_controller_t {
  static constexpr uint8_t cmd_id = 0x30;
};

struct can_msg_enter_bootloader_t {
  static constexpr uint8_t cmd_id = 0x31;
};

struct can_msg_erase_flash_t {
  static constexpr uint8_t cmd_id = 0x32;
};

struct can_msg_select_page_t {
  static constexpr uint8_t cmd_id = 0x33;
  uint16_t page_addr;
};

struct can_msg_mem_packet_t {
  static constexpr uint8_t cmd_id = 0x34;
  uint8_t chunk_addr;
  uint16_t page_addr;
  uint8_t flash_bytes[32];
};

struct can_msg_request_mem_packet_t {
  static constexpr uint8_t cmd_id = 0x35;
  uint8_t chunk_addr;
  uint16_t page_addr;
};

struct can_msg_write_flash_t {
  static constexpr uint8_t cmd_id = 0x36;
};

// RESERVED for Extended Command Format (ID range 0b0111_XXXX)

// Custom Command over CAN (ID range 0b1XXY_YYYY)

/// CAN Helper Functions

// FDCAN DLC to payload size
constexpr std::array<size_t, 16> FDCAN_DLC_SIZES = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

struct fdcan_size_t {
  size_t can_size;     // size of CAN frame needed to represent this struct
  size_t padding_size; // amount of padding needed (sizeof(T) - can_size)
  uint8_t dlc;         // data length code for this CAN size
};

// Returns the FDCAN size information for the provided struct
template <typename T> constexpr fdcan_size_t fdcan_size_of() {
  constexpr size_t size = std::is_empty_v<T> ? 0 : sizeof(T);
  static_assert(size <= FDCAN_DLC_SIZES.back(), "Type exceeds maximum FDCAN frame payload (64 bytes).");
  for (size_t dlc = 0; dlc < FDCAN_DLC_SIZES.size(); ++dlc) {
    if (size <= FDCAN_DLC_SIZES[dlc]) {
      const size_t can_size = FDCAN_DLC_SIZES[dlc];
      return {can_size, can_size - size, static_cast<uint8_t>(dlc)};
    }
  }
  return {0, 0, 0}; // will never be reached because of static_assert above
}

constexpr uint8_t STATE_ANY = 0;

struct can_func {
  uint8_t cmd_id;
  std::function<void(CanMsg)> func;
  uint8_t required_state;
};

std::vector<can_func> can_funcs;

// register a function that takes a buffer and a len
template <typename msg_t> void register_CAN_cmd(std::function<void(msg_t)> f, uint8_t required_state = STATE_ANY) {
  std::function<void(CanMsg)> f_internal = [f](CanMsg raw_msg) {
    if (raw_msg.data_length != fdcan_size_of<msg_t>().can_size) {
      // TODO - wrong len handler
      return;
    }

    msg_t msg;
    memcpy(&msg, raw_msg.data, raw_msg.data_length);
    f(msg);
  };
  can_funcs.push_back({msg_t::cmd_id, f_internal, required_state});
}

template <typename msg_t> void register_CAN_cmd(std::function<void()> f, uint8_t required_state = STATE_ANY) {
  static_assert(fdcan_size_of<msg_t>().can_size == 0);
  std::function<void(CanMsg)> f_internal = [f](CanMsg raw_msg) { f(); };
  can_funcs.push_back({msg_t::cmd_id, f_internal, required_state});
}