#pragma once

#include "AMT242AV.h"
#include "MksServo57D.h"

enum valve_movement_mode_t {
  VALVE_MOVEMENT_MODE_POSITION,
  VALVE_MOVEMENT_MODE_CONSTANT_SPEED,
  VALVE_MOVEMENT_MODE_STOPPED
};

class ThrottleValve {
public:
  ThrottleValve(uint16_t motor_can_id, RS485Device &enc_device, uint8_t enc_ID)
      : motor(motor_can_id), encoder(enc_device, enc_ID) {};

  void begin();
  void update(bool log_csv); // update the control loop
  void stop();
  void set_position(float angle);

  void set_speed(int16_t speed);
  bool read_position(float *pos);
  bool zero_encoder();

private:
  MksServo57D motor;
  AMT242AV encoder;

  float target_angle = 0.0;
  uint32_t last_update_ms = 0.0;
  float K = 50.0;
  valve_movement_mode_t mode = VALVE_MOVEMENT_MODE_STOPPED;
  bool log_csv_sticky = false;
};

namespace ThrottleValves {

bool begin();
void stop();
void update(bool log_csv = false); // update all control loops
void set_angles_ox_fu(float ox_angle, float fu_angle);

} // namespace ThrottleValves