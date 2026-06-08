#pragma once

#include <Arduino.h>

#define PRINT_BUFFER_SIZE 1024
#define READ_BUFFER_SIZE 1024

template <typename BaseSerial> class CommsSerial_t : public BaseSerial {
public:
  using BaseSerial::BaseSerial; // inherit constructors

  char readbuf[READ_BUFFER_SIZE];

  // read until a newline, handling backspaces as needed
  char *readline() {
    int read_pos = 0;

    while (true) {
      if (BaseSerial::available()) {
        char new_char = BaseSerial::read();
        if (new_char == '\b') {
          if (read_pos > 0) {
            read_pos -= 1;
          }
        } else if (new_char == '\n') {
          readbuf[read_pos] = '\0';
          break;
        } else {
          readbuf[read_pos] = new_char;
          read_pos += 1;
          if (read_pos == READ_BUFFER_SIZE) {
            read_pos = 0; // prevent overflow
          }
        }
      }
    }

    return readbuf;
  }

  template <typename... Args> int scanf(const char *format, Args... args) {
    char *buffer = readline();
    return sscanf(buffer, format, args...);
  }

  // adds printf support
  template <typename... Args> void printf(const char *format, Args... args) {
    char buffer[PRINT_BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), format, args...);
    BaseSerial::print(buffer);
  }

  // recursive multiprint
  template <typename T> void mprint(const T &t) { // base case
    BaseSerial::print(t);
  }

  // "multi-print"
  template <typename T, typename... Args> void mprint(const T &t, const Args &...args) { // recursive case. all of this is done at compile time.
    BaseSerial::print(t);
    mprint(args...);
  }

  template <typename... Args> void mprintln(const Args &...args) {
    mprint(args...);
    BaseSerial::println();
  }
};

extern CommsSerial_t<HardwareSerial> HW_CommsSerial;
extern CommsSerial_t<USBSerial> USB_CommsSerial;

#define CommsSerial HW_CommsSerial