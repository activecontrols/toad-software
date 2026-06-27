#include <Arduino.h>

#include "CommsSerial.h"
#include "f4_prog_pins.h"

CommsSerial_t<USBSerial> USB_CommsSerial;
CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX);
CommsSerial_t<HardwareSerial> HW_FallbackSerial(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX);

SPIClass PROG_SPI(PIN_PROG_SPI_MOSI, PIN_PROG_SPI_MISO, PIN_PROG_SPI_SCK);

prog_id_t prog_type;

void setup() {
  // All shared interfaces are begun here.

  // Use same baud rate on all Comm Serials for consistency.
  USB_CommsSerial.begin(RADIO_BAUD);
  HW_CommsSerial.begin(RADIO_BAUD);
  HW_FallbackSerial.begin(RADIO_BAUD);

  PROG_SPI.begin();

  pinMode(PIN_PROG_ID, INPUT);
  prog_type = (digitalRead(PIN_PROG_ID) == PROG_ID_FLIGHT_CONTROLLER) ? PROG_FLIGHT_CONTROLLER : PROG_ENGINE_CONTROLLER;

  delay(3000);

  if (prog_type == PROG_FLIGHT_CONTROLLER) {
    CommsSerial.println("Flight Controller Programmer Started!");
  } else {
    CommsSerial.println("Engine Controller Programmer Started!");
  }
}

void loop() {
  // TODO - prog state machine
  // Wait for CAN cmds, control reset and boot
  // Load flash files and flash over SPI
}