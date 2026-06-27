#include "PressureSensors.h"
#include "CommandRouter.h"
#include "CommsSerial.h"
#include "pt_calibration.h"

// Init the ADC on the PT board.
void PT_Board::begin() {
  adc.begin();
}

// Read both PT channels on the ADC, converting using the saved slope/offsets.
// Return the PT readings through the parameters.
// Returns false if a CRC occurs while reading, true otherwise.
bool PT_Board::read_pts(float *pt0_reading, float *pt1_reading) {
  adc_reading_t adc_reading = adc.read_adc();

  // TODO PTs - add preconversion logic to account for voltage level changes
  // probably need to divide by pow(2, 24) as well
  *pt0_reading = adc_reading.ch0 * pt0_slope + pt0_offset;
  *pt1_reading = adc_reading.ch1 * pt1_slope + pt1_offset;

  return adc_reading.crc_ok;
}

namespace PressureSensors {

/* clang-format off */
PT_Board pt_board_1_2   (PT_BOARD_1_2_SPI_BUS,   PIN_PT_BOARD_1_2_CS,   PT_CALIBRATION(1),  PT_CALIBRATION(2));
PT_Board pt_board_3_4   (PT_BOARD_3_4_SPI_BUS,   PIN_PT_BOARD_3_4_CS,   PT_CALIBRATION(3),  PT_CALIBRATION(4));
PT_Board pt_board_5_6   (PT_BOARD_5_6_SPI_BUS,   PIN_PT_BOARD_5_6_CS,   PT_CALIBRATION(5),  PT_CALIBRATION(6));
PT_Board pt_board_7_8   (PT_BOARD_7_8_SPI_BUS,   PIN_PT_BOARD_7_8_CS,   PT_CALIBRATION(7),  PT_CALIBRATION(8));
PT_Board pt_board_9_10  (PT_BOARD_9_10_SPI_BUS,  PIN_PT_BOARD_9_10_CS,  PT_CALIBRATION(9),  PT_CALIBRATION(10));
PT_Board pt_board_11_12 (PT_BOARD_11_12_SPI_BUS, PIN_PT_BOARD_11_12_CS, PT_CALIBRATION(11), PT_CALIBRATION(12));
static_assert(NUM_PT_BOARDS == 6);
/* clang-format on */

// Configures each PT Board.
// Returns false if a PT Board fails a CRC check.
bool begin() {
  pt_board_1_2.begin();
  pt_board_3_4.begin();
  pt_board_5_6.begin();
  pt_board_7_8.begin();
  pt_board_9_10.begin();
  pt_board_11_12.begin();

  pressure_readings_t pt_readings = read_pts();
  if (pt_readings.crc_errors != NO_PT_CRC_ERRS) {
    print_pt_crc_errors(pt_readings);
    return false;
  }

  CommandRouter::add(print_pt_readings, "print_pt", "Print PT readings in psi.");

  return true;
}

// Read pressure value from all PTs, recording CRC errors if they occur.
pressure_readings_t read_pts() {
  pressure_readings_t pt_readings;
  pt_readings.crc_errors = NO_PT_CRC_ERRS;

  // TODO - for performance, consider parallelizing across the busses (only if needed)
  pt_readings.crc_errors |= pt_board_1_2.read_pts(&pt_readings.PT_1, &pt_readings.PT_2) ? NO_PT_CRC_ERRS : PT_BOARD_1_2_CRC_ERR;
  pt_readings.crc_errors |= pt_board_3_4.read_pts(&pt_readings.PT_3, &pt_readings.PT_4) ? NO_PT_CRC_ERRS : PT_BOARD_3_4_CRC_ERR;
  pt_readings.crc_errors |= pt_board_5_6.read_pts(&pt_readings.PT_5, &pt_readings.PT_6) ? NO_PT_CRC_ERRS : PT_BOARD_5_6_CRC_ERR;
  pt_readings.crc_errors |= pt_board_7_8.read_pts(&pt_readings.PT_7, &pt_readings.PT_8) ? NO_PT_CRC_ERRS : PT_BOARD_7_8_CRC_ERR;
  pt_readings.crc_errors |= pt_board_9_10.read_pts(&pt_readings.PT_9, &pt_readings.PT_10) ? NO_PT_CRC_ERRS : PT_BOARD_9_10_CRC_ERR;
  pt_readings.crc_errors |= pt_board_11_12.read_pts(&pt_readings.PT_11, &pt_readings.PT_12) ? NO_PT_CRC_ERRS : PT_BOARD_11_12_CRC_ERR;

  return pt_readings;
}

// Print the pt board numbers that encountered CRC errors.
void print_pt_crc_errors(pressure_readings_t pt_readings) {
  CommsSerial.print("CRC errors were detected on the following PT boards: ");
  for (int i = 0; i < NUM_PT_BOARDS; i++) {
    if (pt_readings.crc_errors & (1 << i)) {
      CommsSerial.printf("%d_%d ", i * 2 + 1, i * 2 + 2);
    }
    CommsSerial.println();
  }
}

// print PT readings in psi.
void print_pt_readings() {
  pressure_readings_t pt_readings = read_pts();

  CommsSerial.print("PT Readings:");
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_1), pt_readings.PT_1);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_2), pt_readings.PT_2);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_3), pt_readings.PT_3);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_4), pt_readings.PT_4);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_5), pt_readings.PT_5);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_6), pt_readings.PT_6);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_7), pt_readings.PT_7);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_8), pt_readings.PT_8);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_9), pt_readings.PT_9);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_10), pt_readings.PT_10);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_11), pt_readings.PT_11);
  CommsSerial.printf("%25s: %6.2f psi\n", STRINGIFY(PT_12), pt_readings.PT_12);

  if (pt_readings.crc_errors != NO_PT_CRC_ERRS) {
    print_pt_crc_errors(pt_readings);
  }
}

} // namespace PressureSensors