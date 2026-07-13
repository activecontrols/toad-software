#include <Arduino.h>

#include "CommsSerial.h"
#include "f4_prog_pins.h"

// Based on https://www.st.com/resource/en/application_note/an4286-how-to-use-spi-protocol-in-bootloader-on-stm32-mcus-stmicroelectronics.pdf
// Citations commented as [pg#]

CommsSerial_t<USBSerial> USB_CommsSerial;
CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX);
CommsSerial_t<HardwareSerial> HW_FallbackSerial(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX);

SPIClass Prog_SPI(PIN_PROG_SPI_MOSI, PIN_PROG_SPI_MISO, PIN_PROG_SPI_SCK);

constexpr uint8_t ACK = 0x79;
constexpr uint8_t NACK = 0x15;
constexpr uint8_t SYNC = 0x5A;
constexpr uint8_t CMD_WriteMem = 0x31;
constexpr uint8_t CMD_EraseMem = 0x44;

// If return value of function is false, return false from the outer function
// TODO - not sure if this is the design pattern we want
#define ExitOnFail(expr) \
  do {                   \
    if (!(expr))         \
      return false;      \
  } while (0)

prog_id_t prog_type;

enum prog_state_t { STATE_IDLE, STATE_PRE_ERASE, STATE_READY, STATE_PAGE_SELECTED } prog_state;

// 2 MB of Flash divided into 512 x 4096 byte virtual pages.
// Chunks for the active page are received 32 bytes at a time, for 128 chunks per page.
// The active page is written out 256 bytes at a time.
constexpr size_t CAN_CHUNK_SIZE = 32;    // bytes
constexpr size_t WRITE_CHUNK_SIZE = 256; // bytes;
constexpr size_t PAGE_CACHE_SIZE = 4096; // bytes
constexpr size_t NUM_CAN_CHUNKS_PER_PAGE = PAGE_CACHE_SIZE / CAN_CHUNK_SIZE;
constexpr size_t NUM_WRITE_CHUNKS_PER_PAGE = PAGE_CACHE_SIZE / WRITE_CHUNK_SIZE;
constexpr size_t NUM_PAGE_CACHES = 512;
static_assert(PAGE_CACHE_SIZE * NUM_PAGE_CACHES == 2097152);

uint32_t active_pg_id;
uint8_t page_cache[PAGE_CACHE_SIZE];
bool chunk_rcv[NUM_CAN_CHUNKS_PER_PAGE];

void setup() {
  // All shared interfaces are begun here.

  // Use same baud rate on all Comm Serials for consistency.
  USB_CommsSerial.begin(RADIO_BAUD);
  HW_CommsSerial.begin(RADIO_BAUD);
  HW_FallbackSerial.begin(RADIO_BAUD);

  // Configure SPI interface and set CS HIGH
  Prog_SPI.begin();
  pinMode(PIN_PROG_SPI_CS, HIGH);

  // Configure BOOT and NRST to default the STM32H7 into normal code execution
  digitalWrite(PIN_H7_BOOT, BOOT_MODE_RUN);
  pinMode(PIN_H7_BOOT, OUTPUT);

  digitalWrite(PIN_H7_NRST, NRST_MODE_RUN);
  pinMode(PIN_H7_NRST, OUTPUT);

  pinMode(PIN_PROG_ID, INPUT);
  prog_type = (digitalRead(PIN_PROG_ID) == PROG_ID_FLIGHT_CONTROLLER) ? PROG_FLIGHT_CONTROLLER : PROG_ENGINE_CONTROLLER;

  delay(3000);

  if (prog_type == PROG_FLIGHT_CONTROLLER) {
    CommsSerial.println("Flight Controller Programmer Started!");
  } else {
    CommsSerial.println("Engine Controller Programmer Started!");
  }

  prog_state = STATE_IDLE;
}

void reset_h7() {
  digitalWrite(PIN_H7_NRST, NRST_MODE_RST);
  delay(1000);
  digitalWrite(PIN_H7_NRST, NRST_MODE_RUN);
}

// Sends ACK frame, returns true if bootloader
// responded with ACK, false otherwise
// TODO - pretty sure sometimes this has to cycle around until ACK
bool spi_ack_frame() {
  Prog_SPI.transfer(0x00);
  uint8_t resp = Prog_SPI.transfer(0x00);
  Prog_SPI.transfer(ACK);

  if (resp == ACK) {
    return true;
  } else if (resp == NACK) {
    return false;
  } else {
    return false;
  }
}

bool enter_bootloader() {
  // Reset the H7 into the bootloader
  pinMode(PIN_H7_BOOT, BOOT_MODE_FLASH);
  reset_h7();

  // Send SYNC byte and get ACK [6]
  pinMode(PIN_PROG_SPI_CS, LOW);

  Prog_SPI.transfer(SYNC);
  ExitOnFail(spi_ack_frame());

  return true;
}

bool erase_memory() {
  Prog_SPI.transfer(SYNC);
  Prog_SPI.transfer(CMD_EraseMem);
  Prog_SPI.transfer(~CMD_EraseMem);
  ExitOnFail(spi_ack_frame());

  // Mass erase [26]
  Prog_SPI.transfer(0xFF);
  Prog_SPI.transfer(0xFF);
  Prog_SPI.transfer(0xFF ^ 0xFF);
  ExitOnFail(spi_ack_frame());

  return true;
}

bool write_memory(uint32_t addr, const uint8_t *bytes, size_t len) {
  Prog_SPI.transfer(SYNC);
  Prog_SPI.transfer(CMD_WriteMem);
  Prog_SPI.transfer(~CMD_WriteMem);
  ExitOnFail(spi_ack_frame());

  uint8_t addr_byte_0 = (addr >> 24) & 0xFF;
  uint8_t addr_byte_1 = (addr >> 16) & 0xFF;
  uint8_t addr_byte_2 = (addr >> 8) & 0xFF;
  uint8_t addr_byte_3 = (addr >> 0) & 0xFF;

  Prog_SPI.transfer(addr_byte_0);
  Prog_SPI.transfer(addr_byte_1);
  Prog_SPI.transfer(addr_byte_2);
  Prog_SPI.transfer(addr_byte_3);
  Prog_SPI.transfer(addr_byte_0 ^ addr_byte_1 ^ addr_byte_2 ^ addr_byte_3);
  ExitOnFail(spi_ack_frame());

  uint8_t chk_sum = (len - 1) & 0xFF;
  Prog_SPI.transfer((len - 1) & 0xFF);
  for (size_t i = 0; i < len; i++) {
    chk_sum ^= bytes[i];
    Prog_SPI.transfer(bytes[i]);
  }
  Prog_SPI.transfer(chk_sum);
  ExitOnFail(spi_ack_frame());

  return true;
}

void loop() {
  // TODO - prog state machine
  // Wait for CAN cmds, control reset and boot
  // Load flash files and flash over SPI

  // can.pump_events() or similar
  // process commands - note that each command must be guaranteed to not halt
  // TODO - figure out how to parse the incoming cmd and payload
  uint8_t cmd = 0x00;

  if (cmd == 0x00) {
    // probably some heartbeat or status thing
  } else if (cmd == 0x00) {
    reset_h7();

  } else if (cmd = 0x00) { // enter bootloader
    if (prog_state == STATE_IDLE) {
      enter_bootloader();
      prog_state = STATE_PRE_ERASE;
    } else {
      // TODO - handle wrong state
    }

  } else if (cmd = 0x00) { // erase
    if (prog_state == STATE_PRE_ERASE) {
      erase_memory();
      prog_state = STATE_READY;
    } else {
      // TODO - handle wrong state
    }

  } else if (cmd = 0x00) { // select page
    if (prog_state == STATE_READY) {
      prog_state = STATE_PAGE_SELECTED;
      active_pg_id = 0x00;
      for (size_t i = 0; i < NUM_CAN_CHUNKS_PER_PAGE; i++) {
        chunk_rcv[i] = false;
      }
    } else {
      // TODO - handle wrong state
    }
  }

  else if (cmd = 0x00) { // write mem
    if (prog_state == STATE_PAGE_SELECTED) {
      bool all_chunk_rcv = true;
      for (size_t i = 0; i < NUM_CAN_CHUNKS_PER_PAGE; i++) {
        if (!chunk_rcv) {
          // TODO - send request message
          all_chunk_rcv = false;
        }
      }

      if (all_chunk_rcv) {
        for (size_t i = 0; i < NUM_WRITE_CHUNKS_PER_PAGE; i++) {
          static_assert(WRITE_CHUNK_SIZE <= 256);
          write_memory(active_pg_id * PAGE_CACHE_SIZE + WRITE_CHUNK_SIZE * i, &page_cache[WRITE_CHUNK_SIZE * i], WRITE_CHUNK_SIZE);
        }
        prog_state = STATE_READY;
      }
    } else {
      // TODO - handle wrong state
    }

  } else if (cmd = 0x00) { // rcv chunk
    // page_cache[] = incoming;
    // chunk_rcv[] = true;

  } else {
    // TODO
  }
}