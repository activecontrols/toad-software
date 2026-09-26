#include "CommandRouter.h"
#include "MCP48xx.h"
#include "ec_pins.h"

#define PIN_ZI_SPI_SCK PI1
#define PIN_ZI_SPI_MISO PI2
#define PIN_ZI_SPI_MOSI PI3

SPIClass zi_spi(PIN_ZI_SPI_MOSI, PIN_ZI_SPI_MISO, PIN_ZI_SPI_SCK);
MCP4822 dac(zi_spi, PIN_ZUCROW_BOARD_CS);

namespace ZucrowInterface {

void set_voltages(const char *args) {
  float lox_v, ipa_v;
  sscanf(args, "%f %f", &lox_v, &ipa_v);
  lox_v = min(1, max(0, lox_v)); // clamp
  ipa_v = min(1, max(0, ipa_v)); // clamp

  int va = lox_v * 4096; // remap pos from 0 to 1, then multiply by  4096 for 12 bit resolution
  int vb = ipa_v * 4096;
  dac.setVoltageA(va);
  dac.setVoltageB(vb);
  dac.updateDAC();
}

void begin() {
  pinMode(PIN_ZUCROW_BOARD_CS, OUTPUT);
  digitalWrite(PIN_ZUCROW_BOARD_CS, HIGH);
  zi_spi.begin();

  // The channels are turned off at startup so we need to turn the channel we need on
  dac.turnOnChannelA();
  dac.turnOnChannelB();

  // We configure the channels in low gain (2.5 V max)
  dac.setGainA(MCP4822::Low);
  dac.setGainB(MCP4822::Low);

  dac.updateDAC();

  CommandRouter::add(set_voltages, "set_voltages", "set_voltages x y");
}
} // namespace ZucrowInterface
