#pragma once

#include "pid_diagram.h"

#define PT_FU_01_IDX 0
#define PT_N2_01_IDX 1
#define PT_N2_02_IDX 2
#define PT_O2_01_IDX 3
#define TC_N2_01_IDX 4
#define TC_O2_01_IDX 5
#define PT_FU_02_IDX 6
#define PT_O2_02_IDX 7
#define TC_FU_01_IDX 8
#define TC_O2_02_IDX 9
#define PT_FU_04_IDX 10

#define BV_N2_01_IDX 0
#define BV_N2_02_IDX 1
#define SV_N2_01_IDX 2
#define SV_N2_02_IDX 3
#define SV_N2_03_IDX 4
#define SV_N2_04_IDX 5
#define SV_N2_05_IDX 6
#define SV_N2_06_IDX 7
#define SV_N2_07_IDX 8
#define SV_N2_08_IDX 9
#define BV_O2_01_IDX 10
#define BV_O2_02_IDX 11
#define BV_O2_03_IDX 12
#define BV_O2_04_IDX 13
#define SV_O2_01_IDX 14
#define BV_FU_01_IDX 15
#define BV_FU_03_IDX 16
#define BV_FU_04_IDX 17
#define SV_FU_01_IDX 18

extern float sensor_readings[NUMBER_OF_INSTRUMENTS];
extern bool valve_states[NUMBER_OF_VALVES];

void init_fluids_data();
void deinit_fluids_data();
void fluids_data_periodic();