#pragma once

// Host-side stand-in for the stm32duino core, used only by [env:native] tests.
// Pin and SPI activity is recorded in fake_events so tests can check ordering.

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>

// Matches the strict types the real core uses with __COMPAT_H__ defined.
enum PinStatus { LOW = 0, HIGH = 1 };
enum PinMode { INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN };

#define NUM_FAKE_PINS 16

enum FakeEventType { EV_PIN_MODE, EV_DIGITAL_WRITE, EV_SPI_BEGIN, EV_SPI_TRANSFER16, EV_SPI_END };

struct FakeEvent {
  FakeEventType type;
  uint32_t pin; // unused for SPI events
  uint32_t value;
};

inline std::vector<FakeEvent> fake_events;

// Level driven on an output, or the level a test wants an input to read.
inline PinStatus fake_levels[NUM_FAKE_PINS];

// stm32duino only enables a GPIO port's clock inside pinMode(), so a digitalWrite() to a pin that
// was never configured is dropped by the hardware. Modelled per pin rather than per port.
inline bool fake_configured[NUM_FAKE_PINS];

inline void fake_reset() {
  fake_events.clear();
  for (int i = 0; i < NUM_FAKE_PINS; i++) {
    fake_levels[i] = LOW;
    fake_configured[i] = false;
  }
}

inline void pinMode(uint32_t pin, PinMode mode) {
  fake_configured[pin] = true;
  fake_events.push_back({EV_PIN_MODE, pin, (uint32_t)mode});
}

inline void digitalWrite(uint32_t pin, PinStatus level) {
  if (fake_configured[pin]) {
    fake_levels[pin] = level;
  }
  fake_events.push_back({EV_DIGITAL_WRITE, pin, (uint32_t)level});
}

inline PinStatus digitalRead(uint32_t pin) {
  return fake_levels[pin];
}

inline void delay(uint32_t) {}
