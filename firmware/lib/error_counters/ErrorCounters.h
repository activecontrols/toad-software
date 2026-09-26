#pragma once

#include <stdint.h>

// TODO - add to telemetry and UI
namespace ErrorCounters {

extern uint8_t error_counters[];

enum error_counter_t {
  pt_crc,           // incremented every time a PT board read has a CRC error (potentially multiple per flight loop)
  cmd_buf_overflow, // command buffer overflows while parsing a command
  NUM_ERR_COUNTERS
};

void begin();
void zero_all();
void increment(error_counter_t ec);
void zero(error_counter_t ec);

} // namespace ErrorCounters
