#include <Arduino.h>

#ifndef UltramotionActuator_H
#define UltramotionActuator_H

//
// Forward declare rather than #include "fdcan_toad.h" here - a reference is
// all this header needs, and fdcan_toad.h drags in the STM32 HAL headers,
// which callers of this file shouldn't have to pull in just to see these
// prototypes.
class CAN;

struct telem {
  uint32_t status_word = 0;            // A-D: Status word byte
  uint32_t latch_high_status_word = 0; // K-N: Latched high copy of status word byte
  uint32_t latch_low_status_word = 0;  // O-R: Latched low copy of status word byte
  uint32_t unitID = 0;                 // p-s: unitID (CAN ID)

  uint16_t avg_motor_current = 0;      // E-F: Average motor current over telemetry interval (0 to 32767)
  uint16_t abs_servo_cylinder_pos = 0; // G-H: Servo Cylinder position, absolute encoder value (0 to 65535)
  uint16_t rel_servo_cylinder_pos = 0; // I-J: Position converted to input range (pMin to pMax)
  uint8_t phys_stop_pos = 0;           // S: 8-bit position between physical stops (rPos to ePos)
  uint8_t motor_current_avg_16 = 0;    // T: 8-bit motor current 16-sample average of last 16 ms (0 to 255)
  uint8_t bus_voltage = 0;             // U: 8-bit bus voltage 0 VDC to +50 VDC (0 to 255)
  uint8_t motor_current_avg = 0;       // V: 8-bit average motor current over telemetry interval (0 to 255)
  uint8_t max_motor_current_8_bit = 0; // W: 8-bit max motor current over telemetry interval (0 to 255)
  int8_t signed_PCB_temp_sensor = 0;   // X: 8-bit signed integer PCB temp sensor C (-50 to +127)
  uint8_t unsigned_PCB_temp_sensor =
      0; // Y: 8-bit unsigned PCB temp sensor in deg_C where 0 = -50deg_C and 200 = +150deg_C (0 to 200)
  uint8_t PCB_relative_humidity = 0;     // Z: 8-bit PCB relative humidity % (0 to 100)
  uint16_t max_motor_current_16_bit = 0; // mc: Max motor current over telemetry interval (0 to 32767)
  uint16_t target_pos = 0;               // tu: Target position, absolute encoder value

  bool has_printed_status = false;
  bool has_printed_high_status = false;
  bool has_printed_low_status = false;
  bool has_printed_ID = false;
};

#define STATUS_CODE_COUNT 32

// treat all active messages as new
void reset_status_state();

// Decodes a raw CAN frame and prints its contents.
// If `out` is non-null, also copies the decoded telem struct into it so the
// caller (e.g. TVC_Actuators) can act on status/position/fault data instead
// of only logging it.
void parse_CAN_frame(const uint8_t can_msg[], uint8_t msg_length, char decode_str[], uint8_t decode_length,
                     telem *out = nullptr);

// Sends a CAN frame commanding the actuator at `id` to target_pos, over `bus` - data fmt '<>'
void send_target_pos(CAN &bus, uint16_t id, uint16_t target_pos);

// Sends a CAN frame commanding the actuator at `id` to target_pos with a max_torque limit - data fmt '<>()'
void send_target_pos(CAN &bus, uint16_t id, uint16_t target_pos, uint16_t max_torque);

struct tvc_actuator_telemetry_t {
  uint32_t status_word; // latched-high status word, bits 0-23 only (byte 3/N not included by default)
  uint16_t position;    // absolute servo cylinder position
};

tvc_actuator_telemetry_t parse_tvc_telemetry(const uint8_t data[8]);

#endif