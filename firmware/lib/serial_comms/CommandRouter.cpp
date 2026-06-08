#include "CommandRouter.h"
#include <string.h>
#include <vector>

namespace CommandRouter {

// global state for command router
#define MAX_CMD_LEN 1024
uint8_t command_buffer[MAX_CMD_LEN];
size_t command_buffer_pos;
bool escaped; // was the last character an escape char

std::vector<command> commands;

void begin() {
  escaped = false;
  command_buffer_pos = 0;
  commands.clear();
  add(help, "help", "lists all commands or provides help for an individual command");
}

void call_cmd(uint8_t *buffer, size_t buffer_len) {
  // first we need to split the command and args

  size_t command_len = buffer_len;
  uint8_t *arg_buffer = buffer;
  size_t arg_len = 0;

  for (int i = 0; i < buffer_len; i++) {
    if (buffer[i] == ' ') {
      buffer[i] = '\0'; // null terminate so we can strcmp this buffer
      command_len = i;
      arg_buffer = &buffer[command_len + 1];  // +1 b/c of the space
      arg_len = buffer_len - command_len - 1; // -1 b/c of the space
      break;
    }
  }

  if (command_len == buffer_len) { // if we never found a space, make sure argbuffer is pointed at the null terminator
    arg_buffer = &buffer[command_len];
  }

  for (const command &c : commands) {
    if (strcmp(c.name, (char *)buffer) == 0) {
      c.f(arg_buffer, arg_len); // call the function
      return;
    }
  }
  CommsSerial.println("Command not found.");
}

// Call this function in a loop to handle incoming serial messages and process commands
void receive_byte(uint8_t c) {
  if (c == END_CHAR && !escaped) {             // true end
    command_buffer[command_buffer_pos] = '\0'; // null terminate to let us use string tools on this buffer
    int buf_len = command_buffer_pos;
    command_buffer_pos = 0; // allows for nesting commands
    call_cmd(command_buffer, buf_len);
  } else if (c == CR_CHAR && !escaped) { // true carriage return
    // do nothing - we aren't a typewriter, no need to carriage return
  } else if (c == BACKSPACE_CHAR && !escaped) { // true backspace
    if (command_buffer_pos > 0) {
      command_buffer_pos -= 1;
    }
  } else if (c == ESCAPE_CHAR && !escaped) { // true escape start
    escaped = true;
  } else { // literally write the character
    command_buffer[command_buffer_pos] = c;
    command_buffer_pos++;
    escaped = false;
    if (command_buffer_pos == MAX_CMD_LEN) { // we need an extra byte at the end to guarantee a safe null-terminate
      command_buffer_pos = 0;                // something went wrong and we never saw a non-escaped END_CHAR so just go back to 0 and try again
    }
  }
}

// Prints help message for command / lists all commands
void help(const char *command_name) {
  // user just typed "help\n" - we list all commands
  if (strlen(command_name) == 0) {
    CommsSerial.println("All commands: ");
    for (command &c : commands) {
      CommsSerial.println(c.name);
    }
    return;
  }

  // user typed "help command"
  for (const command &c : commands) {
    if (strcmp(c.name, command_name) == 0) {
      CommsSerial.println(c.help);
      return;
    }
  }

  // no command found, so print all commands that start with what user entered
  CommsSerial.println("Similar commands: ");
  for (const command &c : commands) {
    if (strncmp(c.name, command_name, strlen(command_name)) == 0) {
      CommsSerial.println(c.name);
    }
  }
}

// register a function that takes a buffer and a len
void add(std::function<void(const uint8_t *, size_t)> f, const char *name, const char *help) {
  commands.push_back({f, name, help});
}

// register a function that takes a null terminated buffer
void add(std::function<void(const char *)> fstr, const char *name, const char *help) {
  std::function<void(const uint8_t *, size_t)> f = [fcap = fstr](const uint8_t *str, size_t) { fcap((const char *)str); };
  commands.push_back({f, name, help});
}

// register a function that takes a function with no args
void add(std::function<void()> fvoid, const char *name, const char *help) {
  std::function<void(const uint8_t *, size_t)> f = [fcap = fvoid](const uint8_t *str, size_t) { fcap(); };
  commands.push_back({f, name, help});
}

// add a flag that will be set to true when the command is called
void add_flag(bool *flag, const char *name, const char *help) {
  std::function<void(const uint8_t *, size_t)> f = [flagcap = flag](const uint8_t *str, size_t) { *flagcap = true; };
  commands.push_back({f, name, help});
}

} // namespace CommandRouter