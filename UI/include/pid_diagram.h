#pragma once
#include "imgui.h"

#define NUMBER_OF_VALVES 18
#define NUMBER_OF_PID_ITEMS 17
#define NUMBER_OF_INSTRUMENTS 11
#define NUMBER_OF_PIPES 44

// orig from P&ID
// #define PID_COLOR_O2 IM_COL32(0, 176, 240, 255)
// #define PID_COLOR_FU IM_COL32(238, 0, 0, 255)
// #define PID_COLOR_N2 IM_COL32(0, 128, 0, 255)

#define PID_COLOR_O2 IM_COL32(0x19, 0xff, 0xff, 255)
#define PID_COLOR_FU IM_COL32(0xff, 0x36, 0x54, 255)
#define PID_COLOR_N2 IM_COL32(0x32, 0xff, 0x7d, 255)
#define PID_COLOR_VALVE_OPEN IM_COL32(0x32, 0xff, 0x32, 255)
#define PID_COLOR_VALVE_OPEN_EDGE IM_COL32(0x0a, 0x50, 0xa, 255)
#define PID_COLOR_VALVE_CLOSED IM_COL32(0xff, 0x32, 0x32, 255)
#define PID_COLOR_VALVE_CLOSED_EDGE IM_COL32(0x50, 0x0a, 0x0a, 255)
#define PID_COLOR_VALVE_HIGHLIGHT_EDGE IM_COL32(0xff, 0xff, 0xff, 255)
#define PID_COLOR_READOUT_BOX IM_COL32(0x11, 0x1f, 0x38, 255)
#define PID_COLOR_READOUT_BOX_TEXT IM_COL32(0x32, 0xff, 0x32, 255)
#define PID_COLOR_OUTLINE IM_COL32(255, 255, 255, 255)

// element config
#define VALVE_SIZE 30
#define TANK_WIDTH 125
#define TANK_HEIGHT 135
#define LARGE_NOZZEL_WIDTH 100
#define LARGE_NOZZEL_HEIGHT 200
#define SMALL_NOZZEL_WIDTH 25
#define SMALL_NOZZEL_LENGTH 50
#define REG_SIZE 20
#define CHECK_VALVE_SIZE 15

namespace PID_Type {
enum PID_Type { Tank, Large_Nozzle, Small_Nozzle, Igniter, Reg, CheckValve, ManualValve };
}

namespace Instrument_Type {
enum Instrument_Type { PT, TC };
}

namespace Valve_Type {
enum Valve_Type { Solenoid, Ball, Throttle };
}

struct PID_Item {
  PID_Type::PID_Type pid_type;
  const char *name;
  ImVec2 location;
  char orientation; // item specific render data
};

struct PID_Valve {
  Valve_Type::Valve_Type valve_type;
  const char *name;
  ImVec2 location;
  char orientation; // H or V
  ImVec2 label_location;
};

struct PID_Instrument {
  Instrument_Type::Instrument_Type instrument_type;
  const char *name;
  ImVec2 location;
  ImVec2 attach_location;
  char attach_direction;
};

#define NULL_VALVE -1

struct PID_Pipe {
  ImVec2 start;
  ImVec2 end;
  ImColor color;

  int fill_valves[3];  // color set to fade if one of these closed (-1 to disable)
  int purge_valves[3]; // color set to N2 if purge valves are open and fill valves closed

  ImVec2 control_point;
};

struct PID_Diagram {
  PID_Valve valves[NUMBER_OF_VALVES];
  PID_Item pid_items[NUMBER_OF_PID_ITEMS];
  PID_Instrument instruments[NUMBER_OF_INSTRUMENTS];
  PID_Pipe pipes[NUMBER_OF_PIPES];
};

extern PID_Diagram pid_diagram;

void init_diagram();