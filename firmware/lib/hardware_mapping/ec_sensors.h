#pragma once

// Canonical PT Names
#define PT_1 PT_N2_01_tank
#define PT_2 PT_N2_02_reg
#define PT_3 PT_N2_03_service
#define PT_4 PT_O2_01_tank
#define PT_5 PT_O2_02_inj
#define PT_6 PT_O2_03_venturi_upstream
#define PT_7 PT_O2_04_venturi_throat
#define PT_8 PT_FU_01_tank
#define PT_9 PT_FU_02_inj
#define PT_10 PT_FU_03_chamber
#define PT_11 PT_FU_04_igniter
#define PT_12 PT_FU_05_venturi_upstream
static_assert(NUM_PT_BOARDS == 6);

// Canonical TC Names
#define TC_1 TC_N2_01_tank
#define TC_2 TC_O2_01_tank
#define TC_3 TC_FU_01_inj
#define TC_4 TC_FU_02_regen_inlet
static_assert(NUM_TC_CHIPS == 4);

struct pressure_readings_t {
  uint8_t crc_errors;

  // all readings in PSI
  float PT_1;
  float PT_2;
  float PT_3;
  float PT_4;
  float PT_5;
  float PT_6;
  float PT_7;
  float PT_8;
  float PT_9;
  float PT_10;
  float PT_11;
  float PT_12;
};

struct temperature_readings_t {
  // all readings in K
  float TC_1;
  float TC_2;
  float TC_3;
  float TC_4;
};

#define CONCAT_(a, b) a##b
#define CONCAT(a, b) CONCAT_(a, b)
#define PT_CALIBRATION(n) CONCAT(CONCAT(PT_, n), _calibration)