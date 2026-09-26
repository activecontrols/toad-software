#pragma once

// Host-side stand-in for the stm32duino SPI library. Transfers are recorded in fake_events.

#include <Arduino.h>

#define MSBFIRST 1
#define SPI_MODE0 0

struct SPISettings {
  SPISettings(uint32_t clock_hz, uint8_t bit_order, uint8_t mode) : clock_hz(clock_hz), bit_order(bit_order), mode(mode) {}
  uint32_t clock_hz;
  uint8_t bit_order;
  uint8_t mode;
};

struct SPIClass {
  void beginTransaction(SPISettings settings) { fake_events.push_back({EV_SPI_BEGIN, 0, settings.clock_hz}); }
  uint16_t transfer16(uint16_t data) {
    fake_events.push_back({EV_SPI_TRANSFER16, 0, data});
    return 0;
  }
  void endTransaction() { fake_events.push_back({EV_SPI_END, 0, 0}); }
};
