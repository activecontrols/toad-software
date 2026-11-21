#include <Arduino.h>

#include "CommandRouter.h"
#include "CommsSerial.h"
#include "prog_pins.h"
#include "toad_can_bus.h"

// TODO after rebasing - include fdcan_toad.h instead
#include <api/CanMsg.h>

// Based on
// https://www.st.com/resource/en/application_note/an4286-how-to-use-spi-protocol-in-bootloader-on-stm32-mcus-stmicroelectronics.pdf
// Citations commented as [pg#]

// If return value of function is false, return false from the outer function
// TODO - not sure if this is the design pattern we want
#define ExitOnFail(expr) \
  do {                   \
    if (!(expr))         \
      return false;      \
  } while (0)

// Meant to be used with std::optional<T>
#define SetOrFail(dest, src) \
  do {                       \
    auto x = (src);          \
    if (!x) {                \
      return false;          \
    }                        \
    (dest) = x.value();      \
  } while (0)

CommsSerial_t<USBSerial> USB_CommsSerial;
CommsSerial_t<Uart> HW_CommsSerial(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX);
CommsSerial_t<Uart> HW_FallbackSerial(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX);

SPIClass Prog_SPI(PIN_PROG_SPI_MOSI, PIN_PROG_SPI_MISO, PIN_PROG_SPI_SCK);

prog_id_t prog_type;

using namespace CAN_Programmer;

enum class prog_state_t {
  IDLE,      // default state - H7 is running
  BOOTLOADER // H7 has entered SPI bootloader
};

prog_state_t prog_state;

// we can receive commands either over CommsSerial or over CAN
enum class command_source_t {
  HOST_SERIAL, // command was sent by operator over serial
  HOST_TOOL,   // command was sent by the upload_tool over serial
  CAN          // command was sent by the upload_tool over CAN
};

// Keep track of which source sent us the command we are currently processing
command_source_t current_command_source = command_source_t::HOST_SERIAL;

void reset_h7();
void enter_bootloader_cmd();

template <msg_error_t::error_code_t err_code, const char *error_text> bool reply_error();
bool reply_success(); // send SUCCESS error to host

template <typename T> bool reply(T data);

void serial_tool_cmd();

uint16_t my_can_id;
void setup() {
  // All shared interfaces are begun here.
  digitalWrite(PIN_PROG_SPI_CS, HIGH);
  pinMode(PIN_PROG_SPI_CS, OUTPUT);

  // Use same baud rate on all Comm Serials for consistency.
  USB_CommsSerial.begin(RADIO_BAUD);
  HW_CommsSerial.begin(RADIO_BAUD);
  HW_FallbackSerial.begin(RADIO_BAUD);

  // Configure SPI interface and set CS HIGH
  Prog_SPI.begin();

  // Configure BOOT and NRST to default the STM32H7 into normal code execution
  pinMode(PIN_H7_BOOT, BOOT_MODE_RUN);

  digitalWrite(PIN_H7_NRST, NRST_OUTPUT_RUN);
  pinMode(PIN_H7_NRST, OUTPUT_OPENDRAIN);

  pinMode(PIN_PROG_ID, INPUT);
  prog_type = (digitalRead(PIN_PROG_ID) == PROG_ID_FLIGHT_CONTROLLER) ? PROG_FLIGHT_CONTROLLER : PROG_ENGINE_CONTROLLER;

  delay(3000);

  if (prog_type == PROG_FLIGHT_CONTROLLER) {
    CommsSerial.println("Flight Controller Programmer Started!");

    my_can_id = CAN_ID_FLIGHT_PROG;
  } else {
    CommsSerial.println("Engine Controller Programmer Started!");

    my_can_id = CAN_ID_ENGINE_PROG;
  }

  CommandRouter::begin();
  CommandRouter::add(reset_h7, "reset_h7");
  CommandRouter::add(enter_bootloader_cmd, "enter_bootloader_cmd");
  CommandRouter::add(serial_tool_cmd, "tool_cmd");

  prog_state = prog_state_t::IDLE;
}

void reset_h7() {
  digitalWrite(PIN_H7_NRST, NRST_OUTPUT_RST);
  delay(50);
  digitalWrite(PIN_H7_NRST, NRST_OUTPUT_RUN);
  delay(50);
}

void exit_bootloader() {
  digitalWrite(PIN_PROG_SPI_CS, HIGH);
  pinMode(PIN_H7_BOOT, BOOT_MODE_RUN);
  reset_h7();
}

bool enter_bootloader() {
  digitalWrite(PIN_PROG_SPI_CS, HIGH);

  // Reset the H7 into the bootloader
  digitalWrite(PIN_H7_BOOT, BOOT_OUTPUT_FLASH);
  pinMode(PIN_H7_BOOT, BOOT_MODE_FLASH);
  reset_h7();

  Prog_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_PROG_SPI_CS, LOW);

  prog_state = prog_state_t::BOOTLOADER;

  return true;
}

void enter_bootloader_cmd() {
  CommsSerial.println("Entering bootloader...");
  bool result = enter_bootloader();
  if (result) {
    CommsSerial.println("Success...");
  } else {
    CommsSerial.println("Failure");
  }
}

template <typename T> void handle_command(T data) {}

bool read_with_timeout(uint8_t *out, size_t count, uint32_t timeout_ms = 250) {
  uint32_t start_ms = millis();

  size_t i = 0;
  while (i < count) {
    if (millis() - start_ms > timeout_ms) {
      return false;
    }

    if (CommsSerial.available()) {
      out[i++] = CommsSerial.read();
    }
  }

  return true;
}

void handle_cmd(uint8_t cmd_id, uint8_t* data, uint8_t len)
{
  switch (cmd_id)
  {
    case msg_reset_t::cmd_id:
      reset_h7();
      reply_success(); // there is no failure mode here
      break;

    case msg_enter_bootloader_t::cmd_id:
      if (enter_bootloader())
      {
        reply_success();
      }
      else
      {
        // This branch shouldn't be possible here - todo get rid of this branch
        static constexpr char err_txt[] = "Enter bootloader failed";
        reply_error<msg_error_t::FAIL, err_txt>();
      }
      break;

    case msg_reset_into_app_t::cmd_id:
      exit_bootloader();
      reply_success();
      break;

    case msg_spi_transfer_t::cmd_id:
      msg_spi_transfer_t command_data = {};
      memcpy(&command_data, data, sizeof(command_data));

      msg_spi_response_t response_data = {};
      response_data.len = command_data.len;

      for (uint8_t i = 0; i < command_data.len; ++i)
      {
        response_data.data[i] = Prog_SPI.transfer(command_data.data[i]);
      }

      reply(response_data);
      break;
  };
}


void serial_tool_cmd() {

  // f returns early (false) only when there is a timeout
  auto f = []() -> bool
  {
    uint8_t len;
    ExitOnFail(read_with_timeout(&len, 1));

    uint8_t cmd_id;
    ExitOnFail(read_with_timeout(&cmd_id, 1));

    static uint8_t struct_buf[64];
    ExitOnFail(read_with_timeout(struct_buf, len));

    handle_cmd(cmd_id, struct_buf, len);

    return true;
  };

  current_command_source = command_source_t::HOST_TOOL;

  if (!f())
  {
    static constexpr char default_err[] = "ERROR: timed out";
    reply_error<msg_error_t::FAIL, default_err>();
  }

  current_command_source = command_source_t::HOST_SERIAL;
}

void loop() {
  while (CommsSerial.available())
  {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}

template <typename T> bool reply(T data) {
  if (current_command_source == command_source_t::HOST_TOOL) {
    // send length[1], cmd_id[1], data to host over serial
    CommsSerial.write(static_cast<uint8_t>(sizeof(T)));
    CommsSerial.write(T::cmd_id);

    // Note: sizeof() evaluates to 1 for empty structs; this is fine as long as host side also looks for 1 byte when
    // receiving an empty struct
    CommsSerial.write(reinterpret_cast<uint8_t*>(&data), sizeof(T));
    return true;
  }
  // todo: transmit over CAN if that was the source of the command

  return true;
}

template <msg_error_t::error_code_t err_code, const char *error_text> bool reply_error() {
  static constexpr size_t len = strlen(error_text);
  static_assert(len < sizeof(msg_error_t::msg), "Error: message is too long!");

  if (current_command_source == command_source_t::HOST_SERIAL) {
    // just print the error message since it's human-readable
    CommsSerial.println(error_text);
    return true;
  }

  // else we either transmit over can or serial the packed struct
  msg_error_t transmit_struct = {.code = err_code};

  strcpy(transmit_struct.msg, error_text);

  return reply(transmit_struct);
}

bool reply_success() // send empty SUCCESS error to host
{
  static constexpr char text[] = "SUCCESS";
  return reply_error<msg_error_t::SUCCESS, text>();
}