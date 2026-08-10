#pragma once

// we try to keep this a pretty clean file because other tools might import to interface with it
// so no windows.h or things like that

// this is a ring buffer to create history graphs
// if packets arrive from earlier to later as ABCDE we store
// [{0 0 0 0} 0 0 0 0] - rs = 0, wp = 0/4
// [A {0 0 0 A} 0 0 0] - rs = 1, wp = 1/5
// [A B {0 0 A B} 0 0] - rs = 2, wp = 2/6
// [A B C {0 A B C} 0] - rs = 3, wp = 3/7
// [{A B C D} A B C D] - rs = 0, wp = 0/4
// [E {B C D E} B C D] - rs = 1, wp = 1/5

#define FLIGHT_HISTORY_LENGTH 1000

// TODO - clean up these names
struct flight_history_t {
  float accel_x[FLIGHT_HISTORY_LENGTH * 2];
  float accel_y[FLIGHT_HISTORY_LENGTH * 2];
  float accel_z[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_yaw[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_pitch[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_roll[FLIGHT_HISTORY_LENGTH * 2];
  float mag_x[FLIGHT_HISTORY_LENGTH * 2];
  float mag_y[FLIGHT_HISTORY_LENGTH * 2];
  float mag_z[FLIGHT_HISTORY_LENGTH * 2];
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
  float gyro_bias_yaw[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_bias_pitch[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_bias_roll[FLIGHT_HISTORY_LENGTH * 2];
  float accel_bias_x[FLIGHT_HISTORY_LENGTH * 2];
  float accel_bias_y[FLIGHT_HISTORY_LENGTH * 2];
  float accel_bias_z[FLIGHT_HISTORY_LENGTH * 2];
  float mag_bias_x[FLIGHT_HISTORY_LENGTH * 2];
  float mag_bias_y[FLIGHT_HISTORY_LENGTH * 2];
  float mag_bias_z[FLIGHT_HISTORY_LENGTH * 2];

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
  float diffy_perc;
  int rtk_status;
  float gps_hor_prec;
  float gps_ver_prec;
  int gps_sat_count;

  // points to the oldest stored data
  int read_start_pos;
  // points to the most recently written data
  int read_end_pos; // always equal to read_start + FLIGHT_HISTORY_LENGTH - 1
  // points to the next location to write (same as read_start)
  int write_pos;
};

extern flight_history_t FlightHistory; // public interface
#define OUT_BUF_SIZE 32768
extern char concat_msg_buf[OUT_BUF_SIZE]; // for serial msgs
