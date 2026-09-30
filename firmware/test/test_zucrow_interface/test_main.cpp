// Host-side tests for ZucrowInterface. Run with: pio test -e native
// The module is compiled unchanged against the fakes in test/fakes.

#include "CommandRouter.h"
#include "ZucrowInterface.h"
#include "ec_pins.h"
#include <math.h>
#include <unity.h>

// Internal to ZucrowInterface.cpp but externally linked, so the tests can reach them.
namespace ZucrowInterface {
extern uint16_t last_count_ox;
extern uint16_t last_count_fu;
uint16_t angle_to_count(float angle_deg);
} // namespace ZucrowInterface

#define CHANNEL_A_WORD 0x3000 // channel A, 1x gain, output active
#define CHANNEL_B_WORD 0xB000 // channel B, 1x gain, output active

void setUp() {
  fake_reset();
  fake_commands.clear();
  CommsSerial.out.clear();
  ZucrowInterface::last_count_ox = 0;
  ZucrowInterface::last_count_fu = 0;
}

void tearDown() {}

// Index of the first event matching type and pin, or -1.
int find_event(FakeEventType type, uint32_t pin) {
  for (size_t i = 0; i < fake_events.size(); i++) {
    if (fake_events[i].type == type && fake_events[i].pin == pin) {
      return i;
    }
  }
  return -1;
}

int count_events(FakeEventType type) {
  int count = 0;
  for (FakeEvent &e : fake_events) {
    count += e.type == type;
  }
  return count;
}

void assert_event(size_t i, FakeEventType type, uint32_t pin, uint32_t value) {
  TEST_ASSERT_TRUE(i < fake_events.size());
  TEST_ASSERT_EQUAL(type, fake_events[i].type);
  TEST_ASSERT_EQUAL(pin, fake_events[i].pin);
  TEST_ASSERT_EQUAL_HEX32(value, fake_events[i].value);
}

// angle_to_count

void test_angle_to_count_nonpositive_is_zero() {
  TEST_ASSERT_EQUAL_UINT16(0, ZucrowInterface::angle_to_count(0.0f));
  TEST_ASSERT_EQUAL_UINT16(0, ZucrowInterface::angle_to_count(-10.0f));
  TEST_ASSERT_EQUAL_UINT16(0, ZucrowInterface::angle_to_count(-INFINITY));
}

void test_angle_to_count_nan_is_zero() {
  TEST_ASSERT_EQUAL_UINT16(0, ZucrowInterface::angle_to_count(NAN));
}

void test_angle_to_count_scale() {
  TEST_ASSERT_EQUAL_UINT16(1024, ZucrowInterface::angle_to_count(22.5f));
  TEST_ASSERT_EQUAL_UINT16(2048, ZucrowInterface::angle_to_count(45.0f));
  TEST_ASSERT_EQUAL_UINT16(3072, ZucrowInterface::angle_to_count(67.5f));
}

void test_angle_to_count_rounds_to_nearest() {
  TEST_ASSERT_EQUAL_UINT16(0, ZucrowInterface::angle_to_count(0.01f)); // 0.46 counts
  TEST_ASSERT_EQUAL_UINT16(1, ZucrowInterface::angle_to_count(0.02f)); // 0.91 counts
}

void test_angle_to_count_clamps_at_full_scale() {
  TEST_ASSERT_EQUAL_UINT16(4095, ZucrowInterface::angle_to_count(90.0f));
  TEST_ASSERT_EQUAL_UINT16(4095, ZucrowInterface::angle_to_count(1000.0f));
  TEST_ASSERT_EQUAL_UINT16(4095, ZucrowInterface::angle_to_count(INFINITY));
}

// send_valve_angles / DAC writes

void test_send_valve_angles_writes_ox_to_a_and_fu_to_b() {
  ZucrowInterface::send_valve_angles(45.0f, 90.0f);

  // Each write: transaction first, then CS low, one 16 bit word, CS high, end transaction.
  TEST_ASSERT_EQUAL(10, fake_events.size());
  assert_event(0, EV_SPI_BEGIN, 0, 4000000);
  assert_event(1, EV_DIGITAL_WRITE, PIN_ZUCROW_BOARD_CS, LOW);
  assert_event(2, EV_SPI_TRANSFER16, 0, CHANNEL_A_WORD | 2048);
  assert_event(3, EV_DIGITAL_WRITE, PIN_ZUCROW_BOARD_CS, HIGH);
  assert_event(4, EV_SPI_END, 0, 0);
  assert_event(5, EV_SPI_BEGIN, 0, 4000000);
  assert_event(6, EV_DIGITAL_WRITE, PIN_ZUCROW_BOARD_CS, LOW);
  assert_event(7, EV_SPI_TRANSFER16, 0, CHANNEL_B_WORD | 4095);
  assert_event(8, EV_DIGITAL_WRITE, PIN_ZUCROW_BOARD_CS, HIGH);
  assert_event(9, EV_SPI_END, 0, 0);

  TEST_ASSERT_EQUAL_UINT16(2048, ZucrowInterface::last_count_ox);
  TEST_ASSERT_EQUAL_UINT16(4095, ZucrowInterface::last_count_fu);
}

void test_send_valve_angles_bad_input_writes_zero() {
  ZucrowInterface::send_valve_angles(NAN, -5.0f);
  assert_event(2, EV_SPI_TRANSFER16, 0, CHANNEL_A_WORD | 0);
  assert_event(7, EV_SPI_TRANSFER16, 0, CHANNEL_B_WORD | 0);
}

// begin()

void test_begin_returns_true_without_spi_traffic() {
  TEST_ASSERT_TRUE(ZucrowInterface::begin());
  TEST_ASSERT_EQUAL(0, count_events(EV_SPI_BEGIN));
  TEST_ASSERT_EQUAL(0, count_events(EV_SPI_TRANSFER16));
}

// The fake drops writes to pins not yet set with pinMode, like the real core, so these check
// that begin() configures each output before driving it.

void test_begin_deselects_dac() {
  ZucrowInterface::begin();
  assert_event(find_event(EV_PIN_MODE, PIN_ZUCROW_BOARD_CS), EV_PIN_MODE, PIN_ZUCROW_BOARD_CS, OUTPUT);
  TEST_ASSERT_EQUAL(HIGH, fake_levels[PIN_ZUCROW_BOARD_CS]);
}

void test_begin_boots_faulted_and_idle() {
  ZucrowInterface::begin();
  assert_event(find_event(EV_PIN_MODE, PIN_ZUCROW_BOARD_DO1), EV_PIN_MODE, PIN_ZUCROW_BOARD_DO1, OUTPUT);
  assert_event(find_event(EV_PIN_MODE, PIN_ZUCROW_BOARD_DO2), EV_PIN_MODE, PIN_ZUCROW_BOARD_DO2, OUTPUT);
  TEST_ASSERT_EQUAL(HIGH, fake_levels[PIN_ZUCROW_BOARD_DO1]);
  TEST_ASSERT_EQUAL(LOW, fake_levels[PIN_ZUCROW_BOARD_DO2]);
}

void test_begin_sets_inputs() {
  ZucrowInterface::begin();
  assert_event(find_event(EV_PIN_MODE, PIN_ZUCROW_BOARD_DI1), EV_PIN_MODE, PIN_ZUCROW_BOARD_DI1, INPUT);
  assert_event(find_event(EV_PIN_MODE, PIN_ZUCROW_BOARD_DI2), EV_PIN_MODE, PIN_ZUCROW_BOARD_DI2, INPUT);
}

void test_begin_leaves_reserved_pins_alone() {
  ZucrowInterface::begin();
  for (FakeEvent &e : fake_events) {
    TEST_ASSERT_NOT_EQUAL(PIN_ZUCROW_BOARD_DI3, e.pin);
    TEST_ASSERT_NOT_EQUAL(PIN_ZUCROW_BOARD_DI4, e.pin);
    TEST_ASSERT_NOT_EQUAL(PIN_ZUCROW_BOARD_EXTRA, e.pin);
  }
}

void test_begin_registers_commands() {
  ZucrowInterface::begin();
  const char *names[] = {"zi_status", "zi_fault", "zi_ok", "zi_run", "zi_idle", "zi_angles", "zi_sweep"};
  TEST_ASSERT_EQUAL(7, fake_commands.size());
  for (size_t i = 0; i < 7; i++) {
    TEST_ASSERT_EQUAL_STRING(names[i], fake_commands[i].name.c_str());
  }
}

// Line polarities: Zucrow pulls low to assert, we drive high to assert.

void test_check_fault_asserted_when_low() {
  fake_levels[PIN_ZUCROW_BOARD_DI1] = LOW;
  TEST_ASSERT_TRUE(ZucrowInterface::check_fault());
  fake_levels[PIN_ZUCROW_BOARD_DI1] = HIGH;
  TEST_ASSERT_FALSE(ZucrowInterface::check_fault());
}

void test_check_sync_running_when_low() {
  fake_levels[PIN_ZUCROW_BOARD_DI2] = LOW;
  TEST_ASSERT_TRUE(ZucrowInterface::check_sync());
  fake_levels[PIN_ZUCROW_BOARD_DI2] = HIGH;
  TEST_ASSERT_FALSE(ZucrowInterface::check_sync());
}

void test_fault_output_levels() {
  ZucrowInterface::begin();
  ZucrowInterface::send_ok();
  TEST_ASSERT_EQUAL(LOW, fake_levels[PIN_ZUCROW_BOARD_DO1]);
  ZucrowInterface::send_fault();
  TEST_ASSERT_EQUAL(HIGH, fake_levels[PIN_ZUCROW_BOARD_DO1]);
  ZucrowInterface::send_ok();
  TEST_ASSERT_EQUAL(LOW, fake_levels[PIN_ZUCROW_BOARD_DO1]);
}

void test_sync_output_levels() {
  ZucrowInterface::begin();
  ZucrowInterface::send_sync(true);
  TEST_ASSERT_EQUAL(HIGH, fake_levels[PIN_ZUCROW_BOARD_DO2]);
  ZucrowInterface::send_sync(false);
  TEST_ASSERT_EQUAL(LOW, fake_levels[PIN_ZUCROW_BOARD_DO2]);
}

// Serial commands

void test_zi_run_and_zi_idle_commands() {
  ZucrowInterface::begin();
  TEST_ASSERT_TRUE(fake_run_command("zi_run"));
  TEST_ASSERT_EQUAL(HIGH, fake_levels[PIN_ZUCROW_BOARD_DO2]);
  TEST_ASSERT_TRUE(fake_run_command("zi_idle"));
  TEST_ASSERT_EQUAL(LOW, fake_levels[PIN_ZUCROW_BOARD_DO2]);
}

void test_zi_angles_command_writes_dac() {
  ZucrowInterface::begin();
  fake_events.clear();
  TEST_ASSERT_TRUE(fake_run_command("zi_angles", "45 22.5"));
  TEST_ASSERT_EQUAL(2, count_events(EV_SPI_TRANSFER16));
  TEST_ASSERT_EQUAL_UINT16(2048, ZucrowInterface::last_count_ox);
  TEST_ASSERT_EQUAL_UINT16(1024, ZucrowInterface::last_count_fu);
}

void test_zi_angles_command_rejects_bad_args() {
  ZucrowInterface::begin();
  fake_events.clear();
  TEST_ASSERT_TRUE(fake_run_command("zi_angles", "45"));
  TEST_ASSERT_EQUAL(0, count_events(EV_SPI_TRANSFER16));
  TEST_ASSERT_NOT_NULL(strstr(CommsSerial.out.c_str(), "Usage: zi_angles"));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_angle_to_count_nonpositive_is_zero);
  RUN_TEST(test_angle_to_count_nan_is_zero);
  RUN_TEST(test_angle_to_count_scale);
  RUN_TEST(test_angle_to_count_rounds_to_nearest);
  RUN_TEST(test_angle_to_count_clamps_at_full_scale);
  RUN_TEST(test_send_valve_angles_writes_ox_to_a_and_fu_to_b);
  RUN_TEST(test_send_valve_angles_bad_input_writes_zero);
  RUN_TEST(test_begin_returns_true_without_spi_traffic);
  RUN_TEST(test_begin_deselects_dac);
  RUN_TEST(test_begin_boots_faulted_and_idle);
  RUN_TEST(test_begin_sets_inputs);
  RUN_TEST(test_begin_leaves_reserved_pins_alone);
  RUN_TEST(test_begin_registers_commands);
  RUN_TEST(test_check_fault_asserted_when_low);
  RUN_TEST(test_check_sync_running_when_low);
  RUN_TEST(test_fault_output_levels);
  RUN_TEST(test_sync_output_levels);
  RUN_TEST(test_zi_run_and_zi_idle_commands);
  RUN_TEST(test_zi_angles_command_writes_dac);
  RUN_TEST(test_zi_angles_command_rejects_bad_args);
  return UNITY_END();
}
