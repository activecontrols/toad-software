#pragma once
#include "ec_pins.h"

#define VALVE_SHORT_NAME_LEN 8 // "SV_N2_01"

// Canonical Valve Names
#define SV_1 SV_N2_01_rcs_pos_1
#define SV_2 SV_N2_02_rcs_pos_2
#define SV_3 SV_N2_03_rcs_neg_1
#define SV_4 SV_N2_04_rcs_neg_2
#define SV_5 SV_N2_05_ox_purge
#define SV_6 SV_N2_06_fu_purge
#define SV_7 SV_N2_07_igniter_purge
#define SV_8 SV_O2_01_ox_igniter
#define SV_9 SV_FU_01_fu_igniter

#define BV_10 BV_N2_01_dump
#define BV_11 BV_N2_02_fill
#define BV_12 BV_O2_01_release
#define BV_13 BV_O2_02_drain
#define BV_14 BV_O2_03_run
#define BV_15 BV_FU_01_release
#define BV_16 BV_FU_03_run

static_assert(NUM_SV_BV_VALVES == 16);

namespace SolenoidValves {
// Note - this must be kept in sync with the list in SolenoidValves.cpp
// It allows referring to a valve number by its canonical name
/* clang-format off */
enum valve_ids { SV_1, SV_2, SV_3, SV_4, SV_5, SV_6, SV_7, SV_8, SV_9, BV_10, BV_11, BV_12, BV_13, BV_14, BV_15, BV_16 };
/* clang-format on */

// A valve state isn't the same as a DO logic level, so use this type to differentiate.
// See valve_state_to_logic_level for more info.
enum valve_state_t { VALVE_CLOSE, VALVE_OPEN };
} // namespace SolenoidValves

// TODO - global valve state with extra bits to check if we have pulsed latch yet, UI state, etc.

// Default Valve Positions
// TODO CONOPS - audit

#define SV_N2_01_rcs_pos_1_default VALVE_CLOSE
#define SV_N2_02_rcs_pos_2_default VALVE_CLOSE
#define SV_N2_03_rcs_neg_1_default VALVE_CLOSE
#define SV_N2_04_rcs_neg_2_default VALVE_CLOSE
#define SV_N2_05_ox_purge_default VALVE_CLOSE
#define SV_N2_06_fu_purge_default VALVE_CLOSE
#define SV_N2_07_igniter_purge_default VALVE_CLOSE
#define SV_O2_01_ox_igniter_default VALVE_CLOSE
#define SV_FU_01_fu_igniter_default VALVE_CLOSE

#define BV_N2_01_dump_default VALVE_OPEN
#define BV_N2_02_fill_default VALVE_CLOSE
#define BV_O2_01_release_default VALVE_OPEN
#define BV_O2_02_drain_default VALVE_CLOSE
#define BV_O2_03_run_default VALVE_CLOSE
#define BV_FU_01_release_default VALVE_OPEN
#define BV_FU_03_run_default VALVE_CLOSE

#define SV_NAME(n) STRINGIFY(CONCAT(SV_, n))
#define BV_NAME(n) STRINGIFY(CONCAT(BV_, n))
#define SV_DEFAULT_STATE(n) CONCAT(CONCAT(SV_, n), _default)
#define BV_DEFAULT_STATE(n) CONCAT(CONCAT(BV_, n), _default)
