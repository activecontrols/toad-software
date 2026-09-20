#include <Arduino.h>

#include "CommandRouter.h"
#include "CommsSerial.h"
#include "PressureSensors.h"
#include "RCS.h"
#include "flash.h"
#include "RS485.h"
#include "SolenoidValves.h"
#include "Spark.h"
#include "TVC_Actuators.h"
#include "TemperatureSensors.h"
#include "ThrottleValves.h"
#include "ValveController.h"
#include "fdcan_toad.h"

// shared interfaces
CommsSerial_t<USBSerial> USB_CommsSerial;
CommsSerial_t<Uart> HW_CommsSerial(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX);
CommsSerial_t<Uart> HW_FallbackSerial(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX);

SPIClass PT_TC_SPI_1(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
SPIClass PT_TC_SPI_3(PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK);

CAN can_tvc(PIN_CAN_TVC_TX, PIN_CAN_TVC_RX);
CAN can_fc(PIN_CAN_FC_TX, PIN_CAN_FC_RX);

bool kill_flag;
bool arm_flag;
void flight_loop();
bool can_begin(void);

void setup() {
  // All shared interfaces are begun here.

  // Use same baud rate on all Comm Serials for consistency.
  USB_CommsSerial.begin(RADIO_BAUD);
  HW_CommsSerial.begin(RADIO_BAUD);
  HW_FallbackSerial.begin(RADIO_BAUD);

  PT_TC_SPI_1.begin();
  PT_TC_SPI_3.begin();

  Flash::begin();

  delay(3000);

  CommsSerial.println("Engine Controller Started!");
  HW_FallbackSerial.println("Enginer Controller Started! [Fallback Serial]");

  ErrorCounters::begin();
  CommandRouter::begin();

  bool all_modules_ok = true;

  all_modules_ok &= can_begin();
  all_modules_ok &= RS485s::begin();
  all_modules_ok &= PressureSensors::begin();
  all_modules_ok &= TemperatureSensors::begin();
  all_modules_ok &= ThrottleValves::begin();
  all_modules_ok &= SolenoidValves::begin();
  all_modules_ok &= TVC_Actuators::begin();
  all_modules_ok &= ValveController::begin();
  all_modules_ok &= Spark::begin();

  if (!all_modules_ok) {
    while (true) {
      CommsSerial.println("At least one module failed to begin(), see errors above.");
      HW_FallbackSerial.println("At least one module failed to begin(). [Fallback Serial]");
      delay(5000);
    }
  }

  CommandRouter::add(flight_loop, "start_flight_loop");
  CommandRouter::add_flag(&kill_flag, "k", "terminate the flight loop early");
  CommandRouter::add_flag(&arm_flag, "arm", "start following a trajectory");
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }

  ThrottleValves::update();
}

// TODO - these?
#define STARTING_VALVE_ANGLE_OX 30
#define STARTING_VALVE_ANGLE_FU 30

void flight_loop() {
  kill_flag = false;
  arm_flag = false;

  // TODO - preflight checks

  // TODO - reset sensors and outputs
  ThrottleValves::set_angles_ox_fu(STARTING_VALVE_ANGLE_OX, STARTING_VALVE_ANGLE_FU);
  TVC_Actuators::set_angles_pitch_yaw(0.0, 0.0);
  RCS::close();

  while (true) {
    while (CommsSerial.available()) {
      CommandRouter::receive_byte(CommsSerial.read());
    }

    if (kill_flag) {
      break;
    }

    // INPUT
    pressure_readings_t pt_readings = PressureSensors::read_pts();
    temperature_readings_t tc_readings = TemperatureSensors::read_tcs();

    // RUN CONTROLLER
    valve_controller_output_t vco = ValveController::get_controller_output(pt_readings, tc_readings);
    float tvc_pitch = 0.0; // TODO - these come from the FC over CAN
    float tvc_yaw = 0.0;
    float rcs_force = 0.0;

    // OUTPUT - only if armed
    if (arm_flag) {
      ThrottleValves::set_angles_ox_fu(vco.ox_angle, vco.fu_angle);
      TVC_Actuators::set_angles_pitch_yaw(tvc_pitch, tvc_yaw);
      RCS::update_rcs_valves(rcs_force);
    }
  }

  // TODO - safe remaining valves
  ThrottleValves::stop();
  RCS::close();
}

/* Platform-dependent can_begin(); because fdcan lib is used by multiple platforms, it's best to keep platform-dependent
 * init here */
bool can_begin(void) {
  bool is_ok = true;

  is_ok &= can_tvc.begin(CanBitRate::BR_500k);
  is_ok &= can_fc.begin(CanBitRate::BR_500k);

  if (is_ok) {
    CommandRouter::add(cmd_can_loopback<can_tvc, can_fc>, "tvc_to_fc_test");
    CommandRouter::add(cmd_can_loopback<can_fc, can_tvc>, "fc_to_tvc_test");
    CommandRouter::add([]() { can_fc.cmd_can_send_test(); }, "can_fc_send_test");
    CommandRouter::add([]() { can_tvc.cmd_can_send_test(); }, "can_tvc_send_test");
    CommandRouter::add([]() { can_fc.cmd_can_receive_test(); }, "can_fc_receive_test");
    CommandRouter::add([]() { can_tvc.cmd_can_receive_test(); }, "can_tvc_receive_test");
  }

  return is_ok;
}