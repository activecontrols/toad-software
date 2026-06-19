#include "ADS131M02.h"

class PT_Board {
public:
  // TODO - offsets will be determined with runtime calibration
  PT_Board(SPIClass spi_bus, int cs_pin, float pt0_slope, float pt0_offset, float pt1_slope, float pt1_offset)
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

void begin();

} // namespace PressureSensors