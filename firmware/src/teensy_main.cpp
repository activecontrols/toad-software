#include <Arduino.h>

#include "CommandRouter.h"
#include "CommsSerial.h"
#include "ThrottleValves.h"

// shared interfaces
CommsSerial_t<usb_serial_class> USB_CommsSerial;

void setup() {
  // All shared interfaces are begun here.

  // Use same baud rate on all Comm Serials for consistency.
  // USB_CommsSerial.begin(RADIO_BAUD);

  delay(3000);

  CommsSerial.println("Engine Controller Started!");

  motorCAN.begin();
  motorCAN.setBaudRate(500000);
  motorCAN.enableMBInterrupts(); // enable interrupts on entire can bus
  motorCAN.onReceive(ThrottleValves::handle_can_msg);

  CommandRouter::begin();

  bool all_modules_ok = true;
  all_modules_ok &= ThrottleValves::begin();

  if (!all_modules_ok) {
    while (true) {
      CommsSerial.println("At least one module failed to begin(), see errors above.");
      delay(5000);
    }
  }
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
}
