#include <Arduino.h>

#include "CommandRouter.h"
#include "CommsSerial.h"
#include "PressureSensors.h"
#include "RCS.h"
#include "SolenoidValves.h"
#include "TVC_Actuators.h"
#include "TemperatureSensors.h"
#include "ThrottleValves.h"
#include "ValveController.h"

// shared interfaces
// CommsSerial_t<USBSerial> USB_CommsSerial;
CommsSerial_t<Uart> HW_CommsSerial(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX);
CommsSerial_t<Uart> HW_FallbackSerial(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX);
// TODO - configure DE pin
Uart RS485_6(PIN_RS485_6_RX, PIN_RS485_6_TX);
Uart RS485_2(PIN_RS485_2_RX, PIN_RS485_2_TX);

// SPIClass PT_TC_SPI_1(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
// SPIClass PT_TC_SPI_3(PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK);

bool kill_flag;
bool arm_flag;
// void flight_loop();


struct mux_entry
{
  Uart& intf;
  pin_size_t pin;
  const char* name;
};

mux_entry rs485_entries[] = 
{
  {
    .intf = TVC_PITCH_RS485_BUS,
    .pin = PIN_TVC_PITCH_SEL,
    .name = "TVC_PITCH",
  },
  {
    .intf = DRV_OX_RS485_BUS,
    .pin = PIN_DRV_OX_SEL,
    .name = "DRV_OX",
  },
  {
    .intf = ENC_OX_RS485_BUS,
    .pin = PIN_ENC_OX_SEL,
    .name = "ENC_OX",
  },
  {
    .intf = TVC_YAW_RS485_BUS,
    .pin = PIN_TVC_YAW_SEL,
    .name = "TVC_YAW",
  },
  {
    .intf = DRV_FU_RS485_BUS,
    .pin = PIN_DRV_FU_SEL,
    .name = "DRV_FU",
  },
  {
    .intf = ENC_FU_RS485_BUS,
    .pin = PIN_ENC_FU_SEL,
    .name = "ENC_FU",
  },
};

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*a))

void cmd_test_mux(const char* args)
{
  mux_entry* entry = nullptr;
  for (mux_entry& entry_it : rs485_entries)
  {
    if (strcmp(args, entry_it.name) == 0)
    {
      entry = &entry_it;
    }
  }
  if (entry == nullptr)
  {
    CommsSerial.println("Usage: text_mux <interface name>");
    return;
  }

  // mux for this interface
  for (int i = 0; i < ARRAYSIZE(rs485_entries); ++i)
  {
    if (rs485_entries + i != entry)
    {
      digitalWrite(rs485_entries[i].pin, LOW);
    }
    else
    {
      digitalWrite(rs485_entries[i].pin, HIGH);
    }
  }

  // transmit data
  entry->intf.write("Hello, world!\n");
  entry->intf.flush();

  // clear selection
  digitalWrite(entry->pin, LOW);
}

void setup() {
  // All shared interfaces are begun here.

  // Use same baud rate on all Comm Serials for consistency.
  // USB_CommsSerial.begin(RADIO_BAUD);
  HW_CommsSerial.begin(RADIO_BAUD);
  HW_FallbackSerial.begin(RADIO_BAUD);

  for (int i = 0; i < ARRAYSIZE(rs485_entries); ++i)
  {
    digitalWrite(rs485_entries[i].pin, LOW);
    pinMode(rs485_entries[i].pin, OUTPUT);
  }

  RS485_6.begin(9600); // TODO - what baud?
  RS485_2.begin(9600);

  // PT_TC_SPI_1.begin();
  // PT_TC_SPI_3.begin();

  delay(3000);

  CommsSerial.println("Engine Controller Started!");
  HW_FallbackSerial.println("Enginer Controller Started! [Fallback Serial]");

  CommandRouter::begin();

  // bool all_modules_ok = true;
  // all_modules_ok &= PressureSensors::begin();
  // all_modules_ok &= TemperatureSensors::begin();
  // all_modules_ok &= ThrottleValves::begin();
  // all_modules_ok &= SolenoidValves::begin();
  // all_modules_ok &= TVC_Actuators::begin();
  // all_modules_ok &= ValveController::begin();

  // if (!all_modules_ok) {
  //   while (true) {
  //     CommsSerial.println("At least one module failed to begin(), see errors above.");
  //     HW_FallbackSerial.println("At least one module failed to begin(), see errors above. [Fallback Serial]");
  //     delay(5000);
  //   }
  // }

  // CommandRouter::add(flight_loop, "start_flight_loop");
  CommandRouter::add_flag(&kill_flag, "k", "terminate the flight loop early");
  CommandRouter::add_flag(&arm_flag, "arm", "start following a trajectory");
  CommandRouter::add(cmd_test_mux, "test_mux", "Usage: text_mux <interface name>");
  
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  while (CommsSerial.available()) {
    CommandRouter::receive_byte(CommsSerial.read());
  }
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // USB_CommsSerial.println("HELLO USB!");
  HW_CommsSerial.println("HELLO HARDWARE!");
  // HW_FallbackSerial.println("HELLO FALLBACK!");
}

// // TODO - these?
// #define STARTING_VALVE_ANGLE_OX 30
// #define STARTING_VALVE_ANGLE_FU 30

// void flight_loop() {
//   kill_flag = false;
//   arm_flag = false;

//   // TODO - preflight checks

//   // TODO - reset sensors and outputs
//   ThrottleValves::set_angles_ox_fu(STARTING_VALVE_ANGLE_OX, STARTING_VALVE_ANGLE_FU);
//   TVC_Actuators::set_angles_pitch_yaw(0.0, 0.0);
//   RCS::close();

//   while (true) {
//     while (CommsSerial.available()) {
//       CommandRouter::receive_byte(CommsSerial.read());
//     }

//     if (kill_flag) {
//       break;
//     }

//     // INPUT
//     pressure_readings_t pt_readings = PressureSensors::read_pts();
//     temperature_readings_t tc_readings = TemperatureSensors::read_tcs();

//     // RUN CONTROLLER
//     valve_controller_output_t vco = ValveController::get_controller_output(pt_readings, tc_readings);
//     float tvc_pitch = 0.0; // TODO - these come from the FC over CAN
//     float tvc_yaw = 0.0;
//     float rcs_force = 0.0;

//     // OUTPUT - only if armed
//     if (arm_flag) {
//       ThrottleValves::set_angles_ox_fu(vco.ox_angle, vco.fu_angle);
//       TVC_Actuators::set_angles_pitch_yaw(tvc_pitch, tvc_yaw);
//       RCS::update_rcs_valves(rcs_force);
//     }
//   }

//   // TODO - safe remaining valves
//   ThrottleValves::stop();
//   RCS::close();
// }
