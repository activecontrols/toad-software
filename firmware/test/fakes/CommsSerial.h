#pragma once

// Host-side stand-in for lib/serial_comms/CommsSerial.h. Output is captured in CommsSerial.out.

#include <Arduino.h>
#include <string>

struct FakeSerial {
  std::string out;

  template <typename... Args> void printf(const char *format, Args... args) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format, args...);
    out += buffer;
  }

  void println(const char *s) {
    out += s;
    out += "\n";
  }
};

inline FakeSerial CommsSerial;
