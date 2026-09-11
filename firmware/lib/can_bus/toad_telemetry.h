#pragma once

#include "ec_sensors.h"
#include "ec_valves.h"

struct gnc_telemetry_t {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;
  float mag_x;
  float mag_y;
  float mag_z;
  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;
  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;

  float state_q_vec_new;
  float state_q_vec_0;
  float state_q_vec_1;
  float state_q_vec_2;
  float state_pos_north;
  float state_pos_west;
  float state_pos_up;
  float state_vel_north;
  float state_vel_west;
  float state_vel_up;
  float gyro_bias_yaw;
  float gyro_bias_pitch;
  float gyro_bias_roll;
  float accel_bias_x;
  float accel_bias_y;
  float accel_bias_z;
  float mag_bias_x;
  float mag_bias_y;
  float mag_bias_z;

  float gimbal_yaw_raw;
  float gimbal_pitch_raw;
  float thrust_N;
  float roll_rad_sec_squared;

  float target_pos_north;
  float target_pos_west;
  float target_pos_up;

  float elapsed_time;
  bool GND_flag;
  bool flight_armed;
  float thrust_perc;
  int rtk_status;
  float gps_hor_prec;
  float gps_ver_prec;
  int gps_sat_count;
};

struct ec_telemetry_t {
  pressure_readings_t pts;
  temperature_readings_t tcs;
  uint32_t valve_states;
  float ox_valve_angle;
  float fu_valve_angle;
  float ox_fill_level;
  float fu_fill_level;
  float n2_fill_level;
};