#include "CommsSerial.h"
#include "PressureSensors.h"
#include <Arduino.h>

void setup() {
  // TODO - begin shared resources, like SPI
  // TODO - begin serials

  PressureSensors::begin();
}

void loop() {}