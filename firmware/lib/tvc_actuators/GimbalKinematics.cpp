#include "GimbalKinematics.h"
#include <math.h>

struct pt {
  float x, y, z;
};

// angle in radians - function determined from rotation matrix
pt rotate_x(pt orig_pt, float theta) {
  pt new_pt;
  new_pt.x = orig_pt.x;
  new_pt.y = orig_pt.y * cos(theta) + orig_pt.z * -sin(theta);
  new_pt.z = orig_pt.y * sin(theta) + orig_pt.z * cos(theta);
  return new_pt;
}

// angle in radians - function determined from rotation matrix
pt rotate_y(pt orig_pt, float theta) {
  pt new_pt;
  new_pt.x = orig_pt.x * cos(theta) + orig_pt.z * sin(theta);
  new_pt.y = orig_pt.y;
  new_pt.z = orig_pt.x * -sin(theta) + orig_pt.z * cos(theta);
  return new_pt;
}

float distance(pt a, pt b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

// TODO - update these
#define BASE_POINT_DIST_FROM_ORIGIN_H 6.25
#define BASE_POINT_DIST_FROM_ORIGIN_V -2
#define ENGINE_POINT_DIST_FROM_ORIGIN_H 3.475
#define ENGINE_POINT_DIST_FROM_ORIGIN_V 10.35
#define BASE_ACTUATOR_LEN 12.6

// uses kinematics to find the 2 actuator lengths given input angles
// input angles in radians
// output lengths in inches (already subtracted from the base actuator length)
void calc_actuator_lengths(float primary_angle, float secondary_angle, float *primary_length, float *secondary_length) {
  float primary_angle_rad = primary_angle * M_PI / 180;
  float secondary_angle_rad = secondary_angle * M_PI / 180;

  pt base_point_p = {0, BASE_POINT_DIST_FROM_ORIGIN_H, BASE_POINT_DIST_FROM_ORIGIN_V};
  pt base_point_s = {BASE_POINT_DIST_FROM_ORIGIN_H, 0, BASE_POINT_DIST_FROM_ORIGIN_V};
  pt engine_point_p = {0, ENGINE_POINT_DIST_FROM_ORIGIN_H, ENGINE_POINT_DIST_FROM_ORIGIN_V};
  pt engine_point_s = {ENGINE_POINT_DIST_FROM_ORIGIN_H, 0, ENGINE_POINT_DIST_FROM_ORIGIN_V};

  engine_point_p = rotate_x(engine_point_p, primary_angle_rad);
  engine_point_p = rotate_y(engine_point_p, secondary_angle_rad);
  engine_point_s = rotate_x(engine_point_s, primary_angle_rad);
  engine_point_s = rotate_y(engine_point_s, secondary_angle_rad);

  *primary_length = distance(base_point_p, engine_point_p) - BASE_ACTUATOR_LEN;
  *secondary_length = distance(base_point_s, engine_point_s) - BASE_ACTUATOR_LEN;
}