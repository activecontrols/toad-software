#pragma once

#include <cstdint>

#include "controller_and_estimator.h"

#include "IMU.h"
#include "Mag.h"

namespace Logging {

void begin();

void complete();

void write(uint8_t *data, unsigned int len);

template <typename t> void write(t x) {
  write((uint8_t *)&x, sizeof(x));
  return;
}

bool is_armed();

void disarm();
}; // namespace Logging