#include <Arduino.h>

#include "CommandRouter.h"
#include "CommsSerial.h"
#include "ErrorCounters.h"
#include "PressureSensors.h"
#include "RCS.h"
#include "RS485.h"
#include "SolenoidValves.h"
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

void setup() {
  // All shared interfaces are begun here.

  // Use same baud rate on all Comm Serials for consistency.
  USB_CommsSerial.begin(RADIO_BAUD);
  HW_CommsSerial.begin(RADIO_BAUD);
  HW_FallbackSerial.begin(RADIO_BAUD);

  PT_TC_SPI_1.begin();
  PT_TC_SPI_3.begin();

  delay(3000);

  CommsSerial.println("Engine Controller Started!");
  HW_FallbackSerial.println("Enginer Controller Started! [Fallback Serial]");

  ErrorCounters::begin();
  CommandRouter::begin();

  bool all_modules_ok = true;

  
  // all_modules_ok &= PressureSensors::begin();
  // all_modules_ok &= TemperatureSensors::begin();
  // all_modules_ok &= RS485s::begin();
  // all_modules_ok &= ThrottleValves::begin();
  // all_modules_ok &= SolenoidValves::begin();
  // all_modules_ok &= TVC_Actuators::begin();
  // all_modules_ok &= ValveController::begin();
  // all_modules_ok &= can_tvc.begin(CanBitRate::BR_500k);
  all_modules_ok &= can_fc.begin(CanBitRate::BR_500k);

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

  CommandRouter::add([]()
  {
      CAN& test_can = can_tvc;

      const uint8_t str[] = "hello123";
      arduino::CanMsg msg(0x55, 8, str);
      
      test_can.write(msg);
  }, "can_loopback_test");


  CommandRouter::add([]()
  {
    static int count = 0;
    while (!CommsSerial.available())
    {
      CanMsg to_send(0x00, 8, (const uint8_t*)"EC->PROG");

      if (can_fc.write(to_send) != 1)
      {
        CommsSerial.println("can_fc.write() failed");
        return;
      }

      uint32_t start_ms = millis();

      while (!can_fc.available())
      {
        if (millis() - start_ms > 500)
        {
          CommsSerial.println("Error: reply timed out");
          return;
        }
      }

      while (can_fc.available())
      {
        CanMsg msg = can_fc.read();

        CommsSerial.printf("(%3d) Received message (0x%08X): %.*s\n", count++, msg.id, msg.data_length, msg.data);
      }

      delay(1000);
    }
  }, "gse_can_test");
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
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
