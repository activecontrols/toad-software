#include <Arduino.h>

#include "CommandRouter.h"
#include "CommsSerial.h"
#include "PressureSensors.h"

// shared interfaces
CommsSerial_t<HardwareSerial> HW_CommsSerial(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX);
CommsSerial_t<HardwareSerial> HW_FallbackSerial(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX);
CommsSerial_t<USBSerial> USB_CommsSerial;
SPIClass PT_TC_SPI_1(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
SPIClass PT_TC_SPI_3(PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK);

void setup() {
  // All shared interfaces are begun here.
  HW_CommsSerial.begin(57600);
  USB_CommsSerial.begin(57600);
  PT_TC_SPI_1.begin();
  PT_TC_SPI_3.begin();

  delay(3000);

  CommsSerial.println("Engine Controller Started!");
  HW_FallbackSerial.println("Enginer Controller Started! [Fallback Serial]");

  bool all_modules_ok = true;
  all_modules_ok &= PressureSensors::begin();

  if (!all_modules_ok) {
    while (true) {
      CommsSerial.println("At least one module failed to begin(), see errors above.");
      HW_FallbackSerial.println("At least one module failed to begin(), see errors above. [Fallback Serial]");
      delay(5000);
    }
  }
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}

void flight_loop() {
  // TODO - preflight checks

  // TODO - reset sensors

  while (true) {
    while (CommsSerial.available()) {
      CommandRouter::receive_byte(CommsSerial.read());
    }
  }
}
