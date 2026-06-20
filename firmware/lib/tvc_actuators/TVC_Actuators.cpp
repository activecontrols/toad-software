#include "TVC_Actuators.h"
#include "GimbalKinematics.h"

namespace TVC_Actuators {

bool begin() {
  return true;
}

// TODO - this function
void set_angles_pitch_yaw(float pitch, float yaw) {
  float pitch_len;
  float yaw_len;
  calc_actuator_lengths(pitch, yaw, &pitch_len, &yaw_len);
}

} // namespace TVC_Actuators