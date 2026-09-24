#include "UltramotionActuator.hpp"
#include "CommsSerial.h"
#include "fdcan_toad.h"

// Changes from the previous version: sends now go through the real fdcan_toad
// build an arduino::CanMsg and call bus.write() directly. Everything else
// (status_codes[], print_new_status_codes(), parse_CAN_byte(),
// print_can_data()) is unchanged.

using namespace arduino;

const char *status_codes[STATUS_CODE_COUNT] = {
    "Position at or beyond retracted physical stop rPos", // 0 - 8
    "Position at or beyond extended physical stop ePos",
    "Position beyond retracted software limit spMin",
    "Position beyond extended software limit spMax",
    "Supply voltage low, motor in COAST (<6.75 VDC, 1 V hysteresis)",
    "Supply voltage high, motor in dynamic brake  (>44.0 VDC, 2 V hysteresis)",
    "Torque output greater than ovTorq limit",
    "Torque command at maxTorq limit",
    "Speed below \xe2\x80\x9cstop\xe2\x80\x9d threshold", // 9 - 16
    "Direction is extend",
    "Position at target (position near target within posWin for posTime)",
    "Following error (position error larger than fErrWin for time period fErrTime)",
    "Command RX error (message not received in [rxTO * 800 \xc2\xb5s])",
    "Telemetry TX error (message not sent over full telemetry interval)",
    "CAN position command input capped at low limit pMin",
    "CAN position command input capped at upper limit pMax",
    "Trajectory move active ", // 17 - 24
    "Heating active",
    "Temperature at PCB greater than ovTemp value",
    "Temperature at PCB less than unTemp value",
    "Relative humidity at PCB greater than ovHumi value",
    "Fatal error in CONFIG.TXT or HARDWARE.TXT",
    "Fault output bit of DRV8323RS (Bridge Driver)",
    "Erroneous warm reset of the CPU has occurred",
    "opMode (CLI = 0, CAN = 1)", // 25 - 32
    "Interpolation enabled ",
    "Heating enabled ",
    "CAN bus module in passive mode ",
    "USB connected",
    "Opto input 1",
    "Opto input 2",
    "Opto input 3"};

uint32_t current_status = 0; // limitation - don't mix latched status with normal status for the same variable
uint32_t current_status_latched_high = 0;
uint32_t current_status_latched_low = 0;

void reset_status_state() {
  current_status = 0;
  current_status_latched_high = 0;
  current_status_latched_low = 0;
}

void print_new_status_codes(uint32_t new_status, uint32_t old_status) {
  uint32_t status_dif = new_status ^ old_status;

  CommsSerial.println("  New Status Messages: ");
  uint32_t current_status_shift = new_status;
  uint32_t status_dif_shift = status_dif;
  for (int i = 0; i < STATUS_CODE_COUNT; i++) {
    if (status_dif_shift & 0x1 && current_status_shift & 0x1) { // select bits, check if message is NEW and ACTIVE
      CommsSerial.print("    ");
      CommsSerial.println(status_codes[i]);
    };
    status_dif_shift = status_dif_shift >> 1;
    current_status_shift = current_status_shift >> 1;
  }
  CommsSerial.println("  END");

  CommsSerial.println("\n  Cleared Status: ");
  current_status_shift = new_status;
  status_dif_shift = status_dif;
  for (int i = 0; i < STATUS_CODE_COUNT; i++) {
    if (status_dif_shift & 0x1 && !(current_status_shift & 0x1)) { // select bits, check if message is NEW and INACTIVE
      CommsSerial.print("    ");
      CommsSerial.println(status_codes[i]);
    };
    status_dif_shift = status_dif_shift >> 1;
    current_status_shift = current_status_shift >> 1;
  }
  CommsSerial.println("  END");
}

void parse_CAN_byte(uint8_t can_msg, char decode_str, telem *current_telem_frame) {
  uint16_t can_msg_16bit = can_msg;
  uint32_t can_msg_32bit = can_msg;

  if (decode_str == 'A') {
    current_telem_frame->status_word += can_msg_32bit;
  } else if (decode_str == 'B') {
    current_telem_frame->status_word += can_msg_32bit << 8;
  } else if (decode_str == 'C') {
    current_telem_frame->status_word += can_msg_32bit << 16;
  } else if (decode_str == 'D') {
    current_telem_frame->status_word += can_msg_32bit << 24;
  } else if (decode_str == 'E') {
    current_telem_frame->avg_motor_current += can_msg_16bit;
  } else if (decode_str == 'F') {
    current_telem_frame->avg_motor_current += can_msg_16bit << 8;
  } else if (decode_str == 'G') {
    current_telem_frame->abs_servo_cylinder_pos += can_msg_16bit;
  } else if (decode_str == 'H') {
    current_telem_frame->abs_servo_cylinder_pos += can_msg_16bit << 8;
  } else if (decode_str == 'I') {
    current_telem_frame->rel_servo_cylinder_pos += can_msg_16bit;
  } else if (decode_str == 'J') {
    current_telem_frame->rel_servo_cylinder_pos += can_msg_16bit << 8;
  } else if (decode_str == 'K') {
    current_telem_frame->latch_high_status_word += can_msg_32bit;
  } else if (decode_str == 'L') {
    current_telem_frame->latch_high_status_word += can_msg_32bit << 8;
  } else if (decode_str == 'M') {
    current_telem_frame->latch_high_status_word += can_msg_32bit << 16;
  } else if (decode_str == 'N') {
    current_telem_frame->latch_high_status_word += can_msg_32bit << 24;
  } else if (decode_str == 'O') {
    current_telem_frame->latch_low_status_word += can_msg_32bit;
  } else if (decode_str == 'P') {
    current_telem_frame->latch_low_status_word += can_msg_32bit << 8;
  } else if (decode_str == 'Q') {
    current_telem_frame->latch_low_status_word += can_msg_32bit << 16;
  } else if (decode_str == 'R') {
    current_telem_frame->latch_low_status_word += can_msg_32bit << 24;
  } else if (decode_str == 'S') {
    current_telem_frame->phys_stop_pos = can_msg;
  } else if (decode_str == 'T') {
    current_telem_frame->motor_current_avg_16 = can_msg;
  } else if (decode_str == 'U') {
    current_telem_frame->bus_voltage = can_msg;
  } else if (decode_str == 'V') {
    current_telem_frame->motor_current_avg = can_msg;
  } else if (decode_str == 'W') {
    current_telem_frame->max_motor_current_8_bit = can_msg;
  } else if (decode_str == 'X') {
    current_telem_frame->signed_PCB_temp_sensor = can_msg;
  } else if (decode_str == 'Y') {
    current_telem_frame->unsigned_PCB_temp_sensor = can_msg;
  } else if (decode_str == 'Z') {
    current_telem_frame->PCB_relative_humidity = can_msg;
  } else if (decode_str == 'm') {
    current_telem_frame->max_motor_current_16_bit += can_msg_16bit;
  } else if (decode_str == 'c') {
    current_telem_frame->max_motor_current_16_bit += can_msg_16bit << 8;
  } else if (decode_str == 'p') {
    current_telem_frame->unitID += can_msg_32bit;
  } else if (decode_str == 'q') {
    current_telem_frame->unitID += can_msg_32bit << 8;
  } else if (decode_str == 'r') {
    current_telem_frame->unitID += can_msg_32bit << 16;
  } else if (decode_str == 's') {
    current_telem_frame->unitID += can_msg_32bit << 24;
  } else if (decode_str == 't') {
    current_telem_frame->target_pos += can_msg_16bit;
  } else if (decode_str == 'u') {
    current_telem_frame->target_pos += can_msg_16bit << 8;
  } else {
    CommsSerial.print("FATAL ERROR - CAN frame contained non-decodable char: ");
    CommsSerial.println(decode_str);
  }
}

void print_can_data(char decode_str, telem *current_telem_frame) {
  switch (decode_str) {
  case 'A':
  case 'B':
  case 'C':
  case 'D':
    if (!current_telem_frame->has_printed_status) {
      CommsSerial.println("Status: ");
      print_new_status_codes(current_telem_frame->status_word, current_status);
      current_status = current_telem_frame->status_word;
    }
    current_telem_frame->has_printed_status = true;
    break;
  case 'E': // prevent dup
  case 'G':
  case 'I':
  case 'm':
  case 't':
    break;
  case 'F': // and E
    CommsSerial.print("Average motor current over telemetry interval: ");
    CommsSerial.println(current_telem_frame->avg_motor_current);
    break;
  case 'H': // and G
    CommsSerial.print("Servo Cylinder position, absolute encoder value: ");
    CommsSerial.println(current_telem_frame->abs_servo_cylinder_pos);
    break;
  case 'J': // and I
    CommsSerial.print("Position converted to input range (pMin to pMax): ");
    CommsSerial.println(current_telem_frame->rel_servo_cylinder_pos);
    break;
  case 'K':
  case 'L':
  case 'M':
  case 'N':
    if (!current_telem_frame->has_printed_high_status) {
      CommsSerial.println("Status (Latched High): ");
      print_new_status_codes(current_telem_frame->latch_high_status_word, current_status_latched_high);
      current_status_latched_high = current_telem_frame->latch_high_status_word;
    }
    current_telem_frame->has_printed_high_status = true;
    break;
  case 'O':
  case 'P':
  case 'Q':
  case 'R':
    if (!current_telem_frame->has_printed_low_status) {
      CommsSerial.println("Status (Latched Low): ");
      print_new_status_codes(current_telem_frame->latch_low_status_word, current_status_latched_low);
      current_status_latched_low = current_telem_frame->latch_low_status_word;
    }
    current_telem_frame->has_printed_low_status = true;
    break;
  case 'S':
    CommsSerial.print("8-bit position between physical stops (rPos to ePos): ");
    CommsSerial.println(current_telem_frame->phys_stop_pos);
    break;
  case 'T':
    CommsSerial.print("8-bit motor current 16-sample average of last 16 ms: ");
    CommsSerial.println(current_telem_frame->motor_current_avg_16);
    break;
  case 'U': {
    CommsSerial.print("8-bit bus voltage 0 VDC to +50 VDC: ");
    float voltage = current_telem_frame->bus_voltage;
    CommsSerial.print(voltage * 50 / 255);
    CommsSerial.println(" VDC");
    break;
  }
  case 'V':
    CommsSerial.print("8-bit average motor current over telemetry interval: ");
    CommsSerial.println(current_telem_frame->motor_current_avg);
    break;
  case 'W':
    CommsSerial.print("8-bit max motor current over telemetry interval: ");
    CommsSerial.println(current_telem_frame->max_motor_current_8_bit);
    break;
  case 'X':
    CommsSerial.print("8-bit signed integer PCB temp sensor C (-50 to +127): ");
    CommsSerial.println(current_telem_frame->signed_PCB_temp_sensor);
    break;
  case 'Y': {
    CommsSerial.print("8-bit unsigned PCB temp sensor: ");
    int pcb_temp = current_telem_frame->unsigned_PCB_temp_sensor;
    CommsSerial.print(pcb_temp - 50);
    CommsSerial.println(" deg C");
    break;
  }
  case 'Z':
    CommsSerial.print("8-bit PCB relative humidity: ");
    CommsSerial.print(current_telem_frame->PCB_relative_humidity);
    CommsSerial.println("%");
    break;
  case 'c': // and m
    CommsSerial.print("Max motor current over telemetry interval: ");
    CommsSerial.println(current_telem_frame->max_motor_current_16_bit);
    break;
  case 'p':
  case 'q':
  case 'r':
  case 's':
    if (!current_telem_frame->has_printed_ID) {
      CommsSerial.print("ID: 0x");
      CommsSerial.println(current_telem_frame->unitID, HEX);
    }
    current_telem_frame->has_printed_ID = true;
    break;
  case 'u': // and t
    CommsSerial.print("Target position, absolute encoder value: ");
    CommsSerial.println(current_telem_frame->target_pos);
    break;
  default:
    CommsSerial.print("FATAL ERROR - CAN frame contained non-decodable char: ");
    CommsSerial.println(decode_str);
    break;
  }
}

void parse_CAN_frame(const uint8_t can_msg[], uint8_t msg_length, char decode_str[], uint8_t decode_length,
                     telem *out) {
  if (msg_length != decode_length) {
    CommsSerial.println("FATAL ERROR - CAN frame and decode code lengths do not match!");
  }

  if (msg_length > 8) {
    CommsSerial.println("FATAL ERROR - CAN frame longer than max!");
  }

  telem current_telem_frame; // values are all set to 0

  for (uint8_t i = 0; i < msg_length; i++) {
    parse_CAN_byte(can_msg[i], decode_str[i], &current_telem_frame);
  }

  CommsSerial.println("CAN FRAME START ++++++++++++++++++++++++++");
  for (uint8_t i = 0; i < msg_length; i++) {
    print_can_data(decode_str[i], &current_telem_frame);
  }
  CommsSerial.println("CAN FRAME END  ---------------------------\n");

  if (out != nullptr) {
    *out = current_telem_frame;
  }
}

tvc_actuator_telemetry_t parse_tvc_telemetry(const uint8_t data[8]) {
  tvc_actuator_telemetry_t out;
  out.status_word = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16); // K,L,M
  out.position = (uint16_t)data[3] | ((uint16_t)data[4] << 8);                                // G,H
  return out;
}

void send_target_pos(CAN &bus, uint16_t id, uint16_t target_pos) {
  // Same byte layout as the original prep_CAN_msg(id, target_pos): 2-byte
  // frame, little-endian target position. toad_can_bus.h documents TOAD's
  // CAN IDs as 11-bit standard, so CanStandardId (not CanExtendedId) is used.
  uint8_t data[2] = {
      static_cast<uint8_t>(target_pos & 0xFF),
      static_cast<uint8_t>(target_pos >> 8),
  };
  CanMsg msg(CanStandardId(id), sizeof(data), data);
  bus.write(msg);
}

void send_target_pos(CAN &bus, uint16_t id, uint16_t target_pos, uint16_t max_torque) {
  uint8_t data[4] = {
      static_cast<uint8_t>(target_pos & 0xFF),
      static_cast<uint8_t>(target_pos >> 8),
      static_cast<uint8_t>(max_torque & 0xFF),
      static_cast<uint8_t>(max_torque >> 8),
  };
  CanMsg msg(CanStandardId(id), sizeof(data), data);
  bus.write(msg);
}