#define IMGUI_DEFINE_MATH_OPERATORS
#include "fluids_data.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "time.h"
#include "ui.h"
#include "ui_components.h"
#include "ui_graphics.h"

float fill_level = 0.5;

#define VALVE_CLICK_TIME_THRESHOLD 5 // seconds
int last_clicked_valve = NULL_VALVE;
time_t last_clicked_valve_time;

void fluids_panel() {
  ImGui::Begin(FLUIDS_PANEL);

  ImGui::PushFont(NULL, 24);
  ImGui::SeparatorText("TOAD Valve Control");
  ImGui::PopFont();

  ImVec2 canvasOrigin = ImGui::GetCursorScreenPos();
  ImVec2 diagram_offset = canvasOrigin + ImVec2(400, 300);
  ImDrawList *dl = ImGui::GetWindowDrawList();

  ImVec2 canvasSize = {1400, 1100};

  dl->PushClipRect(canvasOrigin, canvasOrigin + canvasSize, true);

  // Invisible button to capture input for the whole canvas
  ImGui::InvisibleButton("canvas", canvasSize);
  ImGui::PushFont(NULL, 12); // reduce font size for diagram

  // draw pipes first so other items end up on top
  for (int i = 0; i < NUMBER_OF_PIPES; i++) {
    if (i == 8) { // handle overlaps
      ImColor bk_color = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];

      ImVec2 skip_center = ImVec2(525, 375) + diagram_offset;
      dl->AddRectFilled(skip_center - ImVec2(10, 10), skip_center + ImVec2(10, 10), bk_color);

      skip_center = ImVec2(0, 375) + diagram_offset;
      dl->AddRectFilled(skip_center - ImVec2(10, 10), skip_center + ImVec2(10, 10), bk_color);
    }

    PID_Pipe ppipe = pid_diagram.pipes[i];
    ImColor color = ppipe.color;

    bool can_fill = true;
    bool can_purge = false;
    bool should_purge = true;

    for (int i = 0; i < 3; i++) {
      if (ppipe.fill_valves[i] != NULL_VALVE && !valve_states[ppipe.fill_valves[i]]) {
        can_fill = false;
      }
      if (ppipe.purge_valves[i] != NULL_VALVE && valve_states[ppipe.purge_valves[i]]) {
        can_purge = true;
      }
      if (ppipe.purge_valves[i] != NULL_VALVE && !valve_states[ppipe.purge_valves[i]]) {
        should_purge = false;
      }
    }

    if (!can_fill) {
      if (can_purge && should_purge) {
        color = PID_COLOR_N2;
      } else {
        color = AdjustBrightness(color, 0.5);
      }
    }

    if (ppipe.start.x != ppipe.end.x && ppipe.start.y != ppipe.end.y) {
      dl->AddLine(ppipe.start + diagram_offset, ppipe.control_point + diagram_offset, color, 4);
      dl->AddLine(ppipe.control_point + diagram_offset, ppipe.end + diagram_offset, color, 4);
    } else {
      dl->AddLine(ppipe.start + diagram_offset, ppipe.end + diagram_offset, color, 4);
    }
  }

  if (time(NULL) - last_clicked_valve_time > VALVE_CLICK_TIME_THRESHOLD) {
    last_clicked_valve = NULL_VALVE;
  }

  for (int i = 0; i < NUMBER_OF_VALVES; i++) {
    PID_Valve valve = pid_diagram.valves[i];
    ImVec2 center = valve.location + diagram_offset;

    bool hovered = MouseInValveHitbox(center, valve.orientation);
    if (hovered) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
      if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && !ImGui::IsMouseDragPastThreshold(ImGuiMouseButton_Left)) {
        if (last_clicked_valve == i) { // TODO - check if state hasn't changed since we clicked
          valve_states[i] = !valve_states[i];
          last_clicked_valve = NULL_VALVE;
        } else {
          last_clicked_valve = i;
          last_clicked_valve_time = time(NULL);
        }
      }
    }

    if (last_clicked_valve == i) {
      hovered = true;
    }

    if (valve.valve_type == Valve_Type::Solenoid) {
      DrawValve(center, valve.orientation, valve.name, valve.label_location + diagram_offset, valve_states[i], hovered);
    } else if (valve.valve_type == Valve_Type::Ball) {
      DrawBallValve(center, valve.orientation, valve.name, valve.label_location + diagram_offset, valve_states[i], hovered);
    } else if (valve.valve_type == Valve_Type::Throttle) {
      DrawThrottleValve(center, valve.orientation, valve.name, valve.label_location + diagram_offset, valve_states[i], hovered);
    }
  }

  for (int i = 0; i < NUMBER_OF_PID_ITEMS; i++) {
    PID_Item pitem = pid_diagram.pid_items[i];

    ImVec2 center = pitem.location + diagram_offset;
    if (pitem.pid_type == PID_Type::Tank) {
      ImColor col;
      if (pitem.orientation == 'O') {
        col = PID_COLOR_O2;
      }
      if (pitem.orientation == 'F') {
        col = PID_COLOR_FU;
      }
      if (pitem.orientation == 'N') {
        col = PID_COLOR_N2;
      }

      DrawTank(center, pitem.name, col, fill_level);
    } else if (pitem.pid_type == PID_Type::Large_Nozzle) {
      DrawLargeNozzle(center);
    } else if (pitem.pid_type == PID_Type::Small_Nozzle) {
      DrawSmallNozzle(center, pitem.orientation);
    } else if (pitem.pid_type == PID_Type::Igniter) {
      DrawIgniter(center);
    } else if (pitem.pid_type == PID_Type::Reg) {
      DrawReg(center, pitem.name);
    } else if (pitem.pid_type == PID_Type::CheckValve) {
      DrawCheckValve(center, pitem.orientation);
    } else if (pitem.pid_type == PID_Type::ManualValve) {
      DrawManualValve(center, pitem.name);
    }
  }

  for (int i = 0; i < NUMBER_OF_INSTRUMENTS; i++) {
    PID_Instrument psensor = pid_diagram.instruments[i];

    char *unit;
    if (psensor.instrument_type == Instrument_Type::PT) {
      unit = "psia";
    } else if (psensor.instrument_type == Instrument_Type::TC) {
      unit = "K";
    }

    DrawReadout(psensor.location + diagram_offset, psensor.attach_location + diagram_offset, psensor.name, unit, sensor_readings[i], psensor.attach_direction);
  }

  dl->PopClipRect();
  ImGui::PopFont();

  if (last_clicked_valve != NULL_VALVE) {
    char *new_state;
    if (valve_states[last_clicked_valve]) {
      new_state = "CLOSE";
    } else {
      new_state = "OPEN";
    }
    ImGui::Dummy(ImVec2(100, 0));
    ImGui::SameLine();
    ImGui::Text("Click again to %s %s", new_state, pid_diagram.valves[last_clicked_valve].name);
  }

  ImGui::End();
}