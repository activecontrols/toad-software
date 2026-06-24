#pragma once

#include "SPI.h"

struct adc_reading_t {
  uint32_t status_reg;
  int32_t ch0;
  int32_t ch1;
  bool crc_ok;
};

class ADS131M02 {
public:
  ADS131M02(SPIClass spi_bus, unsigned int cs_pin) : spi_bus(spi_bus), cs_pin(cs_pin) {};
  void begin();
  adc_reading_t read_adc();

private:
  uint32_t transact_word(uint32_t cmd, uint8_t *crc_buf);

  SPIClass spi_bus;
  unsigned int cs_pin;
};