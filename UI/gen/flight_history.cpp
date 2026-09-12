
#include "flight_history.h"

flight_history_t FlightHistory;

int read_start_pos; // points to the oldest stored data
int read_end_pos; // points to the most recently written data, always equal to read_start + FLIGHT_HISTORY_LENGTH - 1
int write_pos; // points to the next location to write (same as read_start)

void init_fh() {
  write_pos = 0;
  read_start_pos = write_pos;
  read_end_pos = read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}

void commit_packet(gnc_telemetry_t packet) {
  FlightHistory.accel_x[write_pos] = packet.accel_x;
  FlightHistory.accel_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_x;
  FlightHistory.accel_y[write_pos] = packet.accel_y;
  FlightHistory.accel_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_y;
  FlightHistory.accel_z[write_pos] = packet.accel_z;
  FlightHistory.accel_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_z;
  FlightHistory.gyro_yaw[write_pos] = packet.gyro_yaw;
  FlightHistory.gyro_yaw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_yaw;
  FlightHistory.gyro_pitch[write_pos] = packet.gyro_pitch;
  FlightHistory.gyro_pitch[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_pitch;
  FlightHistory.gyro_roll[write_pos] = packet.gyro_roll;
  FlightHistory.gyro_roll[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_roll;
  FlightHistory.mag_x[write_pos] = packet.mag_x;
  FlightHistory.mag_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_x;
  FlightHistory.mag_y[write_pos] = packet.mag_y;
  FlightHistory.mag_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_y;
  FlightHistory.mag_z[write_pos] = packet.mag_z;
  FlightHistory.mag_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_z;
  FlightHistory.gps_pos_north[write_pos] = packet.gps_pos_north;
  FlightHistory.gps_pos_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_pos_north;
  FlightHistory.gps_pos_west[write_pos] = packet.gps_pos_west;
  FlightHistory.gps_pos_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_pos_west;
  FlightHistory.gps_pos_up[write_pos] = packet.gps_pos_up;
  FlightHistory.gps_pos_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_pos_up;
  FlightHistory.gps_vel_north[write_pos] = packet.gps_vel_north;
  FlightHistory.gps_vel_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_vel_north;
  FlightHistory.gps_vel_west[write_pos] = packet.gps_vel_west;
  FlightHistory.gps_vel_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_vel_west;
  FlightHistory.gps_vel_up[write_pos] = packet.gps_vel_up;
  FlightHistory.gps_vel_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_vel_up;
  FlightHistory.state_q_vec_new[write_pos] = packet.state_q_vec_new;
  FlightHistory.state_q_vec_new[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_new;
  FlightHistory.state_q_vec_0[write_pos] = packet.state_q_vec_0;
  FlightHistory.state_q_vec_0[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_0;
  FlightHistory.state_q_vec_1[write_pos] = packet.state_q_vec_1;
  FlightHistory.state_q_vec_1[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_1;
  FlightHistory.state_q_vec_2[write_pos] = packet.state_q_vec_2;
  FlightHistory.state_q_vec_2[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_2;
  FlightHistory.state_pos_north[write_pos] = packet.state_pos_north;
  FlightHistory.state_pos_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_pos_north;
  FlightHistory.state_pos_west[write_pos] = packet.state_pos_west;
  FlightHistory.state_pos_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_pos_west;
  FlightHistory.state_pos_up[write_pos] = packet.state_pos_up;
  FlightHistory.state_pos_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_pos_up;
  FlightHistory.state_vel_north[write_pos] = packet.state_vel_north;
  FlightHistory.state_vel_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_vel_north;
  FlightHistory.state_vel_west[write_pos] = packet.state_vel_west;
  FlightHistory.state_vel_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_vel_west;
  FlightHistory.state_vel_up[write_pos] = packet.state_vel_up;
  FlightHistory.state_vel_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_vel_up;
  FlightHistory.gyro_bias_yaw[write_pos] = packet.gyro_bias_yaw;
  FlightHistory.gyro_bias_yaw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_bias_yaw;
  FlightHistory.gyro_bias_pitch[write_pos] = packet.gyro_bias_pitch;
  FlightHistory.gyro_bias_pitch[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_bias_pitch;
  FlightHistory.gyro_bias_roll[write_pos] = packet.gyro_bias_roll;
  FlightHistory.gyro_bias_roll[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_bias_roll;
  FlightHistory.accel_bias_x[write_pos] = packet.accel_bias_x;
  FlightHistory.accel_bias_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_bias_x;
  FlightHistory.accel_bias_y[write_pos] = packet.accel_bias_y;
  FlightHistory.accel_bias_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_bias_y;
  FlightHistory.accel_bias_z[write_pos] = packet.accel_bias_z;
  FlightHistory.accel_bias_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_bias_z;
  FlightHistory.mag_bias_x[write_pos] = packet.mag_bias_x;
  FlightHistory.mag_bias_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_bias_x;
  FlightHistory.mag_bias_y[write_pos] = packet.mag_bias_y;
  FlightHistory.mag_bias_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_bias_y;
  FlightHistory.mag_bias_z[write_pos] = packet.mag_bias_z;
  FlightHistory.mag_bias_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_bias_z;
  FlightHistory.gimbal_yaw_raw[write_pos] = packet.gimbal_yaw_raw;
  FlightHistory.gimbal_yaw_raw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gimbal_yaw_raw;
  FlightHistory.gimbal_pitch_raw[write_pos] = packet.gimbal_pitch_raw;
  FlightHistory.gimbal_pitch_raw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gimbal_pitch_raw;
  FlightHistory.thrust_N[write_pos] = packet.thrust_N;
  FlightHistory.thrust_N[write_pos + FLIGHT_HISTORY_LENGTH] = packet.thrust_N;
  FlightHistory.roll_rad_sec_squared[write_pos] = packet.roll_rad_sec_squared;
  FlightHistory.roll_rad_sec_squared[write_pos + FLIGHT_HISTORY_LENGTH] = packet.roll_rad_sec_squared;
  FlightHistory.target_pos_north[write_pos] = packet.target_pos_north;
  FlightHistory.target_pos_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.target_pos_north;
  FlightHistory.target_pos_west[write_pos] = packet.target_pos_west;
  FlightHistory.target_pos_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.target_pos_west;
  FlightHistory.target_pos_up[write_pos] = packet.target_pos_up;
  FlightHistory.target_pos_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.target_pos_up;
  FlightHistory.elapsed_time[write_pos] = packet.elapsed_time;
  FlightHistory.elapsed_time[write_pos + FLIGHT_HISTORY_LENGTH] = packet.elapsed_time;
  FlightHistory.GND_flag[write_pos] = packet.GND_flag;
  FlightHistory.GND_flag[write_pos + FLIGHT_HISTORY_LENGTH] = packet.GND_flag;
  FlightHistory.flight_armed[write_pos] = packet.flight_armed;
  FlightHistory.flight_armed[write_pos + FLIGHT_HISTORY_LENGTH] = packet.flight_armed;
  FlightHistory.thrust_perc[write_pos] = packet.thrust_perc;
  FlightHistory.thrust_perc[write_pos + FLIGHT_HISTORY_LENGTH] = packet.thrust_perc;
  FlightHistory.rtk_status[write_pos] = packet.rtk_status;
  FlightHistory.rtk_status[write_pos + FLIGHT_HISTORY_LENGTH] = packet.rtk_status;
  FlightHistory.gps_hor_prec[write_pos] = packet.gps_hor_prec;
  FlightHistory.gps_hor_prec[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_hor_prec;
  FlightHistory.gps_ver_prec[write_pos] = packet.gps_ver_prec;
  FlightHistory.gps_ver_prec[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_ver_prec;
  FlightHistory.gps_sat_count[write_pos] = packet.gps_sat_count;
  FlightHistory.gps_sat_count[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_sat_count;
};

void commit_packet(ec_telemetry_t packet) {
  FlightHistory.pts.crc_errors[write_pos] = packet.pts.crc_errors;
  FlightHistory.pts.crc_errors[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.crc_errors;
  FlightHistory.pts.PT_1[write_pos] = packet.pts.PT_1;
  FlightHistory.pts.PT_1[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_1;
  FlightHistory.pts.PT_2[write_pos] = packet.pts.PT_2;
  FlightHistory.pts.PT_2[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_2;
  FlightHistory.pts.PT_3[write_pos] = packet.pts.PT_3;
  FlightHistory.pts.PT_3[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_3;
  FlightHistory.pts.PT_4[write_pos] = packet.pts.PT_4;
  FlightHistory.pts.PT_4[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_4;
  FlightHistory.pts.PT_5[write_pos] = packet.pts.PT_5;
  FlightHistory.pts.PT_5[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_5;
  FlightHistory.pts.PT_6[write_pos] = packet.pts.PT_6;
  FlightHistory.pts.PT_6[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_6;
  FlightHistory.pts.PT_7[write_pos] = packet.pts.PT_7;
  FlightHistory.pts.PT_7[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_7;
  FlightHistory.pts.PT_8[write_pos] = packet.pts.PT_8;
  FlightHistory.pts.PT_8[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_8;
  FlightHistory.pts.PT_9[write_pos] = packet.pts.PT_9;
  FlightHistory.pts.PT_9[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_9;
  FlightHistory.pts.PT_10[write_pos] = packet.pts.PT_10;
  FlightHistory.pts.PT_10[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_10;
  FlightHistory.pts.PT_11[write_pos] = packet.pts.PT_11;
  FlightHistory.pts.PT_11[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_11;
  FlightHistory.pts.PT_12[write_pos] = packet.pts.PT_12;
  FlightHistory.pts.PT_12[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_12;
  FlightHistory.tcs.TC_1[write_pos] = packet.tcs.TC_1;
  FlightHistory.tcs.TC_1[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_1;
  FlightHistory.tcs.TC_2[write_pos] = packet.tcs.TC_2;
  FlightHistory.tcs.TC_2[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_2;
  FlightHistory.tcs.TC_3[write_pos] = packet.tcs.TC_3;
  FlightHistory.tcs.TC_3[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_3;
  FlightHistory.tcs.TC_4[write_pos] = packet.tcs.TC_4;
  FlightHistory.tcs.TC_4[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_4;
  FlightHistory.tcs.TC_5[write_pos] = packet.tcs.TC_5;
  FlightHistory.tcs.TC_5[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_5;
  FlightHistory.tcs.TC_6[write_pos] = packet.tcs.TC_6;
  FlightHistory.tcs.TC_6[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_6;
  FlightHistory.valve_states[write_pos] = packet.valve_states;
  FlightHistory.valve_states[write_pos + FLIGHT_HISTORY_LENGTH] = packet.valve_states;
  FlightHistory.ox_valve_angle[write_pos] = packet.ox_valve_angle;
  FlightHistory.ox_valve_angle[write_pos + FLIGHT_HISTORY_LENGTH] = packet.ox_valve_angle;
  FlightHistory.fu_valve_angle[write_pos] = packet.fu_valve_angle;
  FlightHistory.fu_valve_angle[write_pos + FLIGHT_HISTORY_LENGTH] = packet.fu_valve_angle;
  FlightHistory.ox_fill_level[write_pos] = packet.ox_fill_level;
  FlightHistory.ox_fill_level[write_pos + FLIGHT_HISTORY_LENGTH] = packet.ox_fill_level;
  FlightHistory.fu_fill_level[write_pos] = packet.fu_fill_level;
  FlightHistory.fu_fill_level[write_pos + FLIGHT_HISTORY_LENGTH] = packet.fu_fill_level;
  FlightHistory.n2_fill_level[write_pos] = packet.n2_fill_level;
  FlightHistory.n2_fill_level[write_pos + FLIGHT_HISTORY_LENGTH] = packet.n2_fill_level;
};

void update_fh_pos() {
  write_pos += 1;
  write_pos %= FLIGHT_HISTORY_LENGTH;
  read_start_pos = write_pos;
  read_end_pos = read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}
