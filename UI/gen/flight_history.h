
#pragma once

#include "toad_telemetry.h"

// this is a ring buffer to create history graphs
// if packets arrive from earlier to later as ABCDE we store
// [{0 0 0 0} 0 0 0 0] - rs = 0, wp = 0/4
// [A {0 0 0 A} 0 0 0] - rs = 1, wp = 1/5
// [A B {0 0 A B} 0 0] - rs = 2, wp = 2/6
// [A B C {0 A B C} 0] - rs = 3, wp = 3/7
// [{A B C D} A B C D] - rs = 0, wp = 0/4
// [E {B C D E} B C D] - rs = 1, wp = 1/5

#define FLIGHT_HISTORY_LENGTH 1000

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
  float gps_pos_north[FLIGHT_HISTORY_LENGTH * 2];
  float gps_pos_west[FLIGHT_HISTORY_LENGTH * 2];
  float gps_pos_up[FLIGHT_HISTORY_LENGTH * 2];
  float gps_vel_north[FLIGHT_HISTORY_LENGTH * 2];
  float gps_vel_west[FLIGHT_HISTORY_LENGTH * 2];
  float gps_vel_up[FLIGHT_HISTORY_LENGTH * 2];
  float state_q_vec_new[FLIGHT_HISTORY_LENGTH * 2];
  float state_q_vec_0[FLIGHT_HISTORY_LENGTH * 2];
  float state_q_vec_1[FLIGHT_HISTORY_LENGTH * 2];
  float state_q_vec_2[FLIGHT_HISTORY_LENGTH * 2];
  float state_pos_north[FLIGHT_HISTORY_LENGTH * 2];
  float state_pos_west[FLIGHT_HISTORY_LENGTH * 2];
  float state_pos_up[FLIGHT_HISTORY_LENGTH * 2];
  float state_vel_north[FLIGHT_HISTORY_LENGTH * 2];
  float state_vel_west[FLIGHT_HISTORY_LENGTH * 2];
  float state_vel_up[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_bias_yaw[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_bias_pitch[FLIGHT_HISTORY_LENGTH * 2];
  float gyro_bias_roll[FLIGHT_HISTORY_LENGTH * 2];
  float accel_bias_x[FLIGHT_HISTORY_LENGTH * 2];
  float accel_bias_y[FLIGHT_HISTORY_LENGTH * 2];
  float accel_bias_z[FLIGHT_HISTORY_LENGTH * 2];
  float mag_bias_x[FLIGHT_HISTORY_LENGTH * 2];
  float mag_bias_y[FLIGHT_HISTORY_LENGTH * 2];
  float mag_bias_z[FLIGHT_HISTORY_LENGTH * 2];
  float gimbal_yaw_raw[FLIGHT_HISTORY_LENGTH * 2];
  float gimbal_pitch_raw[FLIGHT_HISTORY_LENGTH * 2];
  float thrust_N[FLIGHT_HISTORY_LENGTH * 2];
  float roll_rad_sec_squared[FLIGHT_HISTORY_LENGTH * 2];
  float target_pos_north[FLIGHT_HISTORY_LENGTH * 2];
  float target_pos_west[FLIGHT_HISTORY_LENGTH * 2];
  float target_pos_up[FLIGHT_HISTORY_LENGTH * 2];
  float elapsed_time[FLIGHT_HISTORY_LENGTH * 2];
  bool GND_flag[FLIGHT_HISTORY_LENGTH * 2];
  bool flight_armed[FLIGHT_HISTORY_LENGTH * 2];
  float thrust_perc[FLIGHT_HISTORY_LENGTH * 2];
  float diffy_perc[FLIGHT_HISTORY_LENGTH * 2];
  int rtk_status[FLIGHT_HISTORY_LENGTH * 2];
  float gps_hor_prec[FLIGHT_HISTORY_LENGTH * 2];
  float gps_ver_prec[FLIGHT_HISTORY_LENGTH * 2];
  int gps_sat_count[FLIGHT_HISTORY_LENGTH * 2];
  struct {
    uint8_t crc_errors[FLIGHT_HISTORY_LENGTH * 2];
    float PT_1[FLIGHT_HISTORY_LENGTH * 2];
    float PT_2[FLIGHT_HISTORY_LENGTH * 2];
    float PT_3[FLIGHT_HISTORY_LENGTH * 2];
    float PT_4[FLIGHT_HISTORY_LENGTH * 2];
    float PT_5[FLIGHT_HISTORY_LENGTH * 2];
    float PT_6[FLIGHT_HISTORY_LENGTH * 2];
    float PT_7[FLIGHT_HISTORY_LENGTH * 2];
    float PT_8[FLIGHT_HISTORY_LENGTH * 2];
    float PT_9[FLIGHT_HISTORY_LENGTH * 2];
    float PT_10[FLIGHT_HISTORY_LENGTH * 2];
    float PT_11[FLIGHT_HISTORY_LENGTH * 2];
    float PT_12[FLIGHT_HISTORY_LENGTH * 2];
  } pts;
  struct {
    float TC_1[FLIGHT_HISTORY_LENGTH * 2];
    float TC_2[FLIGHT_HISTORY_LENGTH * 2];
    float TC_3[FLIGHT_HISTORY_LENGTH * 2];
    float TC_4[FLIGHT_HISTORY_LENGTH * 2];
    float TC_5[FLIGHT_HISTORY_LENGTH * 2];
    float TC_6[FLIGHT_HISTORY_LENGTH * 2];
  } tcs;
};

extern flight_history_t FlightHistory;

extern int read_start_pos;
extern int read_end_pos;
extern int write_pos;

#define fh_now(x) ((x)[read_end_pos])
#define fh_all(x) (&((x)[read_start_pos]))

void init_fh();
void commit_packet(gnc_telemetry_t packet);
void commit_packet(ec_telemetry_t packet);
void update_fh_pos();
