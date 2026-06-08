#pragma once

#include "CommsSerial.h"
#include <functional>
#include <stdint.h>

struct command {
  std::function<void(const uint8_t *, size_t)> f;
  const char *name;
  const char *help;
};

// character types
#define END_CHAR '\n'
#define CR_CHAR '\r'
#define ESCAPE_CHAR '\\'
#define BACKSPACE_CHAR '\b'

namespace CommandRouter {

void begin();

// Call this function in a loop to handle incoming serial messages and process commands
void receive_byte(uint8_t c);

template <typename S> void send_command(const char *command, S data) {
  uint8_t *data_raw = (uint8_t *)&data;

  CommsSerial.print(command);
  CommsSerial.print(" ");
  for (int i = 0; i < sizeof(data); i++) {
    if (data_raw[i] == ESCAPE_CHAR || data_raw[i] == END_CHAR || data_raw[i] == CR_CHAR || data_raw[i] == BACKSPACE_CHAR) {
      CommsSerial.print(ESCAPE_CHAR);
    }
    CommsSerial.write(data_raw[i]);
  }
  CommsSerial.print(END_CHAR);
}

// Prints help message for command / lists all commands
void help(const char *cmd_name);

// register a function that takes a buffer and a len
void add(std::function<void(const uint8_t *, size_t)> f, const char *name, const char *help = "no help provided");

// register a function that takes a null terminated buffer
void add(std::function<void(const char *)> fstr, const char *name, const char *help = "no help provided");

// register a function that takes a function with no args
void add(std::function<void()> fvoid, const char *name, const char *help = "no help provided");

// add a flag that will be set to true when the command is called
void add_flag(bool *flag, const char *name, const char *help);

} // namespace CommandRouter
