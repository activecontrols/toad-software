#pragma once

// Host-side stand-in for lib/hardware_mapping/ec_pins.h. Only the Zucrow board is mapped, onto
// small fake pin numbers that index fake_levels.

#include <Arduino.h>

#include "SPI.h"

inline SPIClass PT_TC_SPI_1;

#define ZUCROW_BOARD_SPI_BUS PT_TC_SPI_1
#define PIN_ZUCROW_BOARD_CS 0

#define PIN_ZUCROW_BOARD_DO1 1
#define PIN_ZUCROW_BOARD_DO2 2

#define PIN_ZUCROW_BOARD_DI1 3
#define PIN_ZUCROW_BOARD_DI2 4
#define PIN_ZUCROW_BOARD_DI3 5
#define PIN_ZUCROW_BOARD_DI4 6

#define PIN_ZUCROW_BOARD_EXTRA 7
