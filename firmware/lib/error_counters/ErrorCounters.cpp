#include "ErrorCounters.h"
#include <cstddef>

namespace ErrorCounters {

uint8_t error_counters[NUM_ERR_COUNTERS];

void begin() {
  zero_all();
}

void zero_all() {
  for (size_t ec = 0; ec < NUM_ERR_COUNTERS; ec++) {
    error_counters[ec] = 0;
  }
}

// Increment an error counter. This will overflow at >255 increments,
// which is ok as we continually telemeter these values.
void increment(error_counter_t ec) {
  error_counters[ec]++;
}

// Reset an error counter to 0.
void zero(error_counter_t ec) {
  error_counters[ec] = 0;
}

} // namespace ErrorCounters
