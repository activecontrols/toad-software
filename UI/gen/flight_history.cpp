
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
  FlightHistory.gnc.accel_x[write_pos] = packet.accel_x;
  FlightHistory.gnc.accel_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_x;
  FlightHistory.gnc.accel_y[write_pos] = packet.accel_y;
  FlightHistory.gnc.accel_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_y;
  FlightHistory.gnc.accel_z[write_pos] = packet.accel_z;
  FlightHistory.gnc.accel_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_z;
  FlightHistory.gnc.gyro_yaw[write_pos] = packet.gyro_yaw;
  FlightHistory.gnc.gyro_yaw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_yaw;
  FlightHistory.gnc.gyro_pitch[write_pos] = packet.gyro_pitch;
  FlightHistory.gnc.gyro_pitch[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_pitch;
  FlightHistory.gnc.gyro_roll[write_pos] = packet.gyro_roll;
  FlightHistory.gnc.gyro_roll[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_roll;
  FlightHistory.gnc.mag_x[write_pos] = packet.mag_x;
  FlightHistory.gnc.mag_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_x;
  FlightHistory.gnc.mag_y[write_pos] = packet.mag_y;
  FlightHistory.gnc.mag_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_y;
  FlightHistory.gnc.mag_z[write_pos] = packet.mag_z;
  FlightHistory.gnc.mag_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_z;
  FlightHistory.gnc.gps_pos_north[write_pos] = packet.gps_pos_north;
  FlightHistory.gnc.gps_pos_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_pos_north;
  FlightHistory.gnc.gps_pos_west[write_pos] = packet.gps_pos_west;
  FlightHistory.gnc.gps_pos_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_pos_west;
  FlightHistory.gnc.gps_pos_up[write_pos] = packet.gps_pos_up;
  FlightHistory.gnc.gps_pos_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_pos_up;
  FlightHistory.gnc.gps_vel_north[write_pos] = packet.gps_vel_north;
  FlightHistory.gnc.gps_vel_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_vel_north;
  FlightHistory.gnc.gps_vel_west[write_pos] = packet.gps_vel_west;
  FlightHistory.gnc.gps_vel_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_vel_west;
  FlightHistory.gnc.gps_vel_up[write_pos] = packet.gps_vel_up;
  FlightHistory.gnc.gps_vel_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_vel_up;
  FlightHistory.gnc.state_q_vec_new[write_pos] = packet.state_q_vec_new;
  FlightHistory.gnc.state_q_vec_new[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_new;
  FlightHistory.gnc.state_q_vec_0[write_pos] = packet.state_q_vec_0;
  FlightHistory.gnc.state_q_vec_0[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_0;
  FlightHistory.gnc.state_q_vec_1[write_pos] = packet.state_q_vec_1;
  FlightHistory.gnc.state_q_vec_1[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_1;
  FlightHistory.gnc.state_q_vec_2[write_pos] = packet.state_q_vec_2;
  FlightHistory.gnc.state_q_vec_2[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_q_vec_2;
  FlightHistory.gnc.state_pos_north[write_pos] = packet.state_pos_north;
  FlightHistory.gnc.state_pos_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_pos_north;
  FlightHistory.gnc.state_pos_west[write_pos] = packet.state_pos_west;
  FlightHistory.gnc.state_pos_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_pos_west;
  FlightHistory.gnc.state_pos_up[write_pos] = packet.state_pos_up;
  FlightHistory.gnc.state_pos_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_pos_up;
  FlightHistory.gnc.state_vel_north[write_pos] = packet.state_vel_north;
  FlightHistory.gnc.state_vel_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_vel_north;
  FlightHistory.gnc.state_vel_west[write_pos] = packet.state_vel_west;
  FlightHistory.gnc.state_vel_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_vel_west;
  FlightHistory.gnc.state_vel_up[write_pos] = packet.state_vel_up;
  FlightHistory.gnc.state_vel_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.state_vel_up;
  FlightHistory.gnc.gyro_bias_yaw[write_pos] = packet.gyro_bias_yaw;
  FlightHistory.gnc.gyro_bias_yaw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_bias_yaw;
  FlightHistory.gnc.gyro_bias_pitch[write_pos] = packet.gyro_bias_pitch;
  FlightHistory.gnc.gyro_bias_pitch[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_bias_pitch;
  FlightHistory.gnc.gyro_bias_roll[write_pos] = packet.gyro_bias_roll;
  FlightHistory.gnc.gyro_bias_roll[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gyro_bias_roll;
  FlightHistory.gnc.accel_bias_x[write_pos] = packet.accel_bias_x;
  FlightHistory.gnc.accel_bias_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_bias_x;
  FlightHistory.gnc.accel_bias_y[write_pos] = packet.accel_bias_y;
  FlightHistory.gnc.accel_bias_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_bias_y;
  FlightHistory.gnc.accel_bias_z[write_pos] = packet.accel_bias_z;
  FlightHistory.gnc.accel_bias_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.accel_bias_z;
  FlightHistory.gnc.mag_bias_x[write_pos] = packet.mag_bias_x;
  FlightHistory.gnc.mag_bias_x[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_bias_x;
  FlightHistory.gnc.mag_bias_y[write_pos] = packet.mag_bias_y;
  FlightHistory.gnc.mag_bias_y[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_bias_y;
  FlightHistory.gnc.mag_bias_z[write_pos] = packet.mag_bias_z;
  FlightHistory.gnc.mag_bias_z[write_pos + FLIGHT_HISTORY_LENGTH] = packet.mag_bias_z;
  FlightHistory.gnc.gimbal_yaw_raw[write_pos] = packet.gimbal_yaw_raw;
  FlightHistory.gnc.gimbal_yaw_raw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gimbal_yaw_raw;
  FlightHistory.gnc.gimbal_pitch_raw[write_pos] = packet.gimbal_pitch_raw;
  FlightHistory.gnc.gimbal_pitch_raw[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gimbal_pitch_raw;
  FlightHistory.gnc.thrust_N[write_pos] = packet.thrust_N;
  FlightHistory.gnc.thrust_N[write_pos + FLIGHT_HISTORY_LENGTH] = packet.thrust_N;
  FlightHistory.gnc.roll_rad_sec_squared[write_pos] = packet.roll_rad_sec_squared;
  FlightHistory.gnc.roll_rad_sec_squared[write_pos + FLIGHT_HISTORY_LENGTH] = packet.roll_rad_sec_squared;
  FlightHistory.gnc.target_pos_north[write_pos] = packet.target_pos_north;
  FlightHistory.gnc.target_pos_north[write_pos + FLIGHT_HISTORY_LENGTH] = packet.target_pos_north;
  FlightHistory.gnc.target_pos_west[write_pos] = packet.target_pos_west;
  FlightHistory.gnc.target_pos_west[write_pos + FLIGHT_HISTORY_LENGTH] = packet.target_pos_west;
  FlightHistory.gnc.target_pos_up[write_pos] = packet.target_pos_up;
  FlightHistory.gnc.target_pos_up[write_pos + FLIGHT_HISTORY_LENGTH] = packet.target_pos_up;
  FlightHistory.gnc.elapsed_time[write_pos] = packet.elapsed_time;
  FlightHistory.gnc.elapsed_time[write_pos + FLIGHT_HISTORY_LENGTH] = packet.elapsed_time;
  FlightHistory.gnc.GND_flag[write_pos] = packet.GND_flag;
  FlightHistory.gnc.GND_flag[write_pos + FLIGHT_HISTORY_LENGTH] = packet.GND_flag;
  FlightHistory.gnc.flight_armed[write_pos] = packet.flight_armed;
  FlightHistory.gnc.flight_armed[write_pos + FLIGHT_HISTORY_LENGTH] = packet.flight_armed;
  FlightHistory.gnc.thrust_perc[write_pos] = packet.thrust_perc;
  FlightHistory.gnc.thrust_perc[write_pos + FLIGHT_HISTORY_LENGTH] = packet.thrust_perc;
  FlightHistory.gnc.rtk_status[write_pos] = packet.rtk_status;
  FlightHistory.gnc.rtk_status[write_pos + FLIGHT_HISTORY_LENGTH] = packet.rtk_status;
  FlightHistory.gnc.gps_hor_prec[write_pos] = packet.gps_hor_prec;
  FlightHistory.gnc.gps_hor_prec[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_hor_prec;
  FlightHistory.gnc.gps_ver_prec[write_pos] = packet.gps_ver_prec;
  FlightHistory.gnc.gps_ver_prec[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_ver_prec;
  FlightHistory.gnc.gps_sat_count[write_pos] = packet.gps_sat_count;
  FlightHistory.gnc.gps_sat_count[write_pos + FLIGHT_HISTORY_LENGTH] = packet.gps_sat_count;
};

void commit_packet(ec_telemetry_t packet) {
  FlightHistory.ec.pts.crc_errors[write_pos] = packet.pts.crc_errors;
  FlightHistory.ec.pts.crc_errors[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.crc_errors;
  FlightHistory.ec.pts.PT_1[write_pos] = packet.pts.PT_1;
  FlightHistory.ec.pts.PT_1[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_1;
  FlightHistory.ec.pts.PT_2[write_pos] = packet.pts.PT_2;
  FlightHistory.ec.pts.PT_2[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_2;
  FlightHistory.ec.pts.PT_3[write_pos] = packet.pts.PT_3;
  FlightHistory.ec.pts.PT_3[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_3;
  FlightHistory.ec.pts.PT_4[write_pos] = packet.pts.PT_4;
  FlightHistory.ec.pts.PT_4[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_4;
  FlightHistory.ec.pts.PT_5[write_pos] = packet.pts.PT_5;
  FlightHistory.ec.pts.PT_5[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_5;
  FlightHistory.ec.pts.PT_6[write_pos] = packet.pts.PT_6;
  FlightHistory.ec.pts.PT_6[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_6;
  FlightHistory.ec.pts.PT_7[write_pos] = packet.pts.PT_7;
  FlightHistory.ec.pts.PT_7[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_7;
  FlightHistory.ec.pts.PT_8[write_pos] = packet.pts.PT_8;
  FlightHistory.ec.pts.PT_8[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_8;
  FlightHistory.ec.pts.PT_9[write_pos] = packet.pts.PT_9;
  FlightHistory.ec.pts.PT_9[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_9;
  FlightHistory.ec.pts.PT_10[write_pos] = packet.pts.PT_10;
  FlightHistory.ec.pts.PT_10[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_10;
  FlightHistory.ec.pts.PT_11[write_pos] = packet.pts.PT_11;
  FlightHistory.ec.pts.PT_11[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_11;
  FlightHistory.ec.pts.PT_12[write_pos] = packet.pts.PT_12;
  FlightHistory.ec.pts.PT_12[write_pos + FLIGHT_HISTORY_LENGTH] = packet.pts.PT_12;
  FlightHistory.ec.tcs.TC_1[write_pos] = packet.tcs.TC_1;
  FlightHistory.ec.tcs.TC_1[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_1;
  FlightHistory.ec.tcs.TC_2[write_pos] = packet.tcs.TC_2;
  FlightHistory.ec.tcs.TC_2[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_2;
  FlightHistory.ec.tcs.TC_3[write_pos] = packet.tcs.TC_3;
  FlightHistory.ec.tcs.TC_3[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_3;
  FlightHistory.ec.tcs.TC_4[write_pos] = packet.tcs.TC_4;
  FlightHistory.ec.tcs.TC_4[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_4;
  FlightHistory.ec.tcs.TC_5[write_pos] = packet.tcs.TC_5;
  FlightHistory.ec.tcs.TC_5[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_5;
  FlightHistory.ec.tcs.TC_6[write_pos] = packet.tcs.TC_6;
  FlightHistory.ec.tcs.TC_6[write_pos + FLIGHT_HISTORY_LENGTH] = packet.tcs.TC_6;
  FlightHistory.ec.valve_states[write_pos] = packet.valve_states;
  FlightHistory.ec.valve_states[write_pos + FLIGHT_HISTORY_LENGTH] = packet.valve_states;
  FlightHistory.ec.ox_valve_angle[write_pos] = packet.ox_valve_angle;
  FlightHistory.ec.ox_valve_angle[write_pos + FLIGHT_HISTORY_LENGTH] = packet.ox_valve_angle;
  FlightHistory.ec.fu_valve_angle[write_pos] = packet.fu_valve_angle;
  FlightHistory.ec.fu_valve_angle[write_pos + FLIGHT_HISTORY_LENGTH] = packet.fu_valve_angle;
  FlightHistory.ec.ox_fill_level[write_pos] = packet.ox_fill_level;
  FlightHistory.ec.ox_fill_level[write_pos + FLIGHT_HISTORY_LENGTH] = packet.ox_fill_level;
  FlightHistory.ec.fu_fill_level[write_pos] = packet.fu_fill_level;
  FlightHistory.ec.fu_fill_level[write_pos + FLIGHT_HISTORY_LENGTH] = packet.fu_fill_level;
  FlightHistory.ec.n2_fill_level[write_pos] = packet.n2_fill_level;
  FlightHistory.ec.n2_fill_level[write_pos + FLIGHT_HISTORY_LENGTH] = packet.n2_fill_level;
};

void commit_frame(flight_frame_t frame) {
  commit_packet(frame.gnc);
  commit_packet(frame.ec);
};

void update_fh_pos() {
  write_pos += 1;
  write_pos %= FLIGHT_HISTORY_LENGTH;
  read_start_pos = write_pos;
  read_end_pos = read_start_pos + FLIGHT_HISTORY_LENGTH - 1;
}
