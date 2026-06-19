#include "ADS131M02.h"
#include "Arduino.h"

// Datasheet: https://www.ti.com/lit/ds/symlink/ads131m02.pdf
// Citations are indicated as [pg#]

#define DISCARD_CRC nullptr

// Configures the cs_pin as an output.
// The SPI bus is a shared resource and should be initialized outside of this function.
void ADS131M02::begin() {
  pinMode(cs_pin, OUTPUT);
  pinMode(cs_pin, HIGH);
}

// Read from both channels on the ADC.
// Uses CRC to check if data received is valid.
adc_reading_t ADS131M02::read_adc() {
  // Send the "NULL" cmd (all 0s) [39].

  adc_reading_t adc_reading;
  uint8_t crc_buf[9];

  adc_reading.status_reg = transact_word(0x000000, &crc_buf[0]);
  adc_reading.ch0 = transact_word(0x000000, &crc_buf[3]);
  adc_reading.ch1 = transact_word(0x000000, &crc_buf[6]);
  uint32_t rcv_crc = transact_word(0x000000, DISCARD_CRC) & 0xFFFF;

  uint16_t calc_crc = 0xFFFF;
  for (int i = 0; i < 9; i++) {
    calc_crc ^= (uint16_t(crc_buf[i]) << 8);
    for (int j = 0; j < 8; j++) {
      if (calc_crc & 0x8000) {
        calc_crc = (calc_crc << 1) ^ 0x1021;
      } else {
        calc_crc <<= 1;
      }
    }
  }

  // handle negation
  if (adc_reading.ch0 > 0x7FFFFF) {
    adc_reading.ch0 = ((~(adc_reading.ch0) & 0x00FFFFFF) + 1) * -1;
  }
  if (adc_reading.ch1 > 0x7FFFFF) {
    adc_reading.ch1 = ((~(adc_reading.ch1) & 0x00FFFFFF) + 1) * -1;
  }

  adc_reading.crc_ok = calc_crc == rcv_crc;

  return adc_reading;
}

// The ADS131M02 communicates in 24 bit words, grouped into 4 word frames.
// Each frame contains:
//   - response to previous command
//   - CH0 data
//   - CH1 data
//   - CRC
// This function should be called 4 times to process a complete frame.
// It fills crc_buf with the 3 bytes recieved.
uint32_t ADS131M02::transact_word(uint32_t cmd, uint8_t *crc_buf) {
  uint8_t a = spi_bus.transfer((cmd >> 16) & 0xFF);
  uint8_t b = spi_bus.transfer((cmd >> 8) & 0xFF);
  uint8_t c = spi_bus.transfer((cmd >> 0) & 0xFF);

  if (crc_buf != DISCARD_CRC) {
    crc_buf[0] = a;
    crc_buf[1] = b;
    crc_buf[2] = c;
  }

  return (uint32_t(a) << 16) | (uint32_t(b) << 8) | (uint32_t(c) << 0);
}