#pragma once

#include "ADS131M02.h"
#include "ec_pins.h"
#include "ec_sensors.h"

#define NO_PT_CRC_ERRS 0
#define PT_BOARD_1_2_CRC_ERR (1 << 0)
#define PT_BOARD_3_4_CRC_ERR (1 << 1)
#define PT_BOARD_5_6_CRC_ERR (1 << 2)
#define PT_BOARD_7_8_CRC_ERR (1 << 3)
#define PT_BOARD_9_10_CRC_ERR (1 << 4)
#define PT_BOARD_11_12_CRC_ERR (1 << 5)
static_assert(NUM_PT_BOARDS == 6);

class PT_Board {
public:
  // TODO - offsets will be determined with runtime calibration
  PT_Board(SPIClass spi_bus, unsigned int cs_pin, float pt0_slope, float pt0_offset, float pt1_slope, float pt1_offset)
      : adc(spi_bus, cs_pin), pt0_slope(pt0_slope), pt0_offset(pt0_offset), pt1_slope(pt1_slope), pt1_offset(pt1_offset) {}

  void begin();
  bool read_pts(float *pt0_reading, float *pt1_reading);

private:
  ADS131M02 adc;
  float pt0_slope;
  float pt0_offset;
  float pt1_slope;
  float pt1_offset;
};

namespace PressureSensors {

bool begin();
pressure_readings_t read_pts();
void print_pt_crc_errors(pressure_readings_t pt_readings);

} // namespace PressureSensors