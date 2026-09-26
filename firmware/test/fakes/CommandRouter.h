#pragma once

// Host-side stand-in for lib/serial_comms/CommandRouter.h. The add() overloads mirror the real
// ones exactly, so overload resolution (lambdas included) behaves the same as on target.

#include "CommsSerial.h"
#include <functional>
#include <stdint.h>
#include <string>
#include <vector>

struct FakeCommand {
  std::string name;
  std::function<void(const char *)> fstr;
  std::function<void()> fvoid;
};

inline std::vector<FakeCommand> fake_commands;

namespace CommandRouter {

inline void add(std::function<void(const uint8_t *, size_t)>, const char *name, const char * = "no help provided") {
  fake_commands.push_back({name, nullptr, nullptr});
}

inline void add(std::function<void(const char *)> fstr, const char *name, const char * = "no help provided") {
  fake_commands.push_back({name, fstr, nullptr});
}

inline void add(std::function<void()> fvoid, const char *name, const char * = "no help provided") {
  fake_commands.push_back({name, nullptr, fvoid});
}

} // namespace CommandRouter

// Runs a registered command as if it arrived over serial. Returns false if it isn't registered.
inline bool fake_run_command(const char *name, const char *args = "") {
  for (FakeCommand &cmd : fake_commands) {
    if (cmd.name == name) {
      if (cmd.fstr) {
        cmd.fstr(args);
      } else if (cmd.fvoid) {
        cmd.fvoid();
      }
      return true;
    }
  }
  return false;
}
