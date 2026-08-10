#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "math.h"
#include "pid_diagram.h"
#include "stdio.h"
#include <algorithm>

// TODO - lots of anti-aliasing stuff is wrong I think
// and should probably review a lot of the graphics API calls to make sure Robert did them right

void DrawValve(ImVec2 center, char orientation, const char *label, ImVec2 label_center, bool open, bool hovered) {
  ImDrawList *dl = ImGui::GetWindowDrawList();

  ImVec2 tl, bl, tr, br; // define key points of the triangles

  if (orientation == 'H') {
    // true coordinates
    tl = center + ImVec2(-VALVE_SIZE, -VALVE_SIZE * 0.5);
    bl = center + ImVec2(-VALVE_SIZE, VALVE_SIZE * 0.5);
    tr = center + ImVec2(VALVE_SIZE, -VALVE_SIZE * 0.5);
    br = center + ImVec2(VALVE_SIZE, VALVE_SIZE * 0.5);
  } else {
    // rotate coords
    tl = center + ImVec2(-VALVE_SIZE * 0.5, VALVE_SIZE);  // bl
    bl = center + ImVec2(VALVE_SIZE * 0.5, VALVE_SIZE);   // br
    tr = center + ImVec2(-VALVE_SIZE * 0.5, -VALVE_SIZE); // tl
    br = center + ImVec2(VALVE_SIZE * 0.5, -VALVE_SIZE);  // tr
  }

  ImColor fill_color = open ? PID_COLOR_VALVE_OPEN : PID_COLOR_VALVE_CLOSED;
  ImColor edge_color = hovered ? PID_COLOR_VALVE_HIGHLIGHT_EDGE : (open ? PID_COLOR_VALVE_OPEN_EDGE : PID_COLOR_VALVE_CLOSED_EDGE);
  int edge_thk = hovered ? 4 : 2;

  dl->AddTriangleFilled(center, tl, bl, fill_color);
  dl->AddTriangleFilled(center, tr, br, fill_color);

  dl->AddTriangle(center, tl, bl, edge_color, edge_thk);
  dl->AddTriangle(center, tr, br, edge_color, edge_thk);

  ImVec2 textSize = ImGui::CalcTextSize(label);
  dl->AddText(label_center - textSize / 2, PID_COLOR_OUTLINE, label);
}

void DrawBallValve(ImVec2 center, char orientation, const char *label, ImVec2 label_center, bool open, bool hovered) {
  ImDrawList *dl = ImGui::GetWindowDrawList();
  ImColor fill_color = open ? PID_COLOR_VALVE_OPEN : PID_COLOR_VALVE_CLOSED;
  ImColor edge_color = hovered ? PID_COLOR_VALVE_HIGHLIGHT_EDGE : (open ? PID_COLOR_VALVE_OPEN_EDGE : PID_COLOR_VALVE_CLOSED_EDGE);
  int edge_thk = hovered ? 4 : 2;

  DrawValve(center, orientation, label, label_center, open, hovered);
  dl->AddCircleFilled(center, VALVE_SIZE * 0.4, fill_color);
  dl->AddCircle(center, VALVE_SIZE * 0.4, edge_color, 0, edge_thk);
}

void DrawThrottleValve(ImVec2 center, char orientation, const char *label, ImVec2 label_center, bool open, bool hovered) {
  ImDrawList *dl = ImGui::GetWindowDrawList();
  DrawBallValve(center, orientation, label, label_center, open, hovered);

  char text_buf[20];
  snprintf(text_buf, sizeof(text_buf), "%.1lf°", 35.2);

  ImVec2 text_size = ImGui::CalcTextSize(text_buf);
  ImVec2 max_text_size = ImGui::CalcTextSize("88.8°");

  ImVec2 rect_tl = center - max_text_size / 2 - ImVec2(5, 5) + ImVec2(0, 60);
  ImVec2 rect_br = center + max_text_size / 2 + ImVec2(5, 5) + ImVec2(0, 60);

  dl->AddRectFilled(rect_tl, rect_br, PID_COLOR_READOUT_BOX, 5);
  dl->AddRect(rect_tl, rect_br, PID_COLOR_OUTLINE, 5);
  dl->AddText(center - text_size / 2 + ImVec2(0, 60), PID_COLOR_READOUT_BOX_TEXT, text_buf);
}

void DrawReg(ImVec2 center, const char *label) {
  ImDrawList *dl = ImGui::GetWindowDrawList();

  ImVec2 tl = center + ImVec2(-REG_SIZE, -REG_SIZE * 0.5);
  ImVec2 bl = center + ImVec2(-REG_SIZE, REG_SIZE * 0.5);
  ImVec2 tr = center + ImVec2(REG_SIZE, -REG_SIZE * 0.5);
  ImVec2 br = center + ImVec2(REG_SIZE, REG_SIZE * 0.5);

  ImColor fill_color = PID_COLOR_OUTLINE;
  ImColor edge_color = PID_COLOR_OUTLINE;
  int edge_thk = 2;

  dl->AddTriangleFilled(center, bl, tl, fill_color);
  dl->AddTriangleFilled(center, tr, br, fill_color);

  float reg_offset = 10;
  float reg_circle_rad = 8;
  dl->PathArcTo(center + ImVec2(0, -reg_offset), reg_circle_rad, M_PI, 2 * M_PI);
  dl->PathStroke(fill_color);
  dl->AddLine(center + ImVec2(-reg_circle_rad, -reg_offset), center + ImVec2(reg_circle_rad, -reg_offset), fill_color);
  dl->AddLine(center, center + ImVec2(0, -reg_offset), fill_color);
  dl->AddLine(center, center + ImVec2(VALVE_SIZE / 2, -reg_offset - reg_circle_rad / 2), fill_color);
  dl->AddLine(center + ImVec2(reg_circle_rad / 1.25, -reg_offset - reg_circle_rad / 2), center + ImVec2(VALVE_SIZE / 2, -reg_offset - reg_circle_rad / 2), fill_color);

  ImVec2 textSize = ImGui::CalcTextSize(label);
  dl->AddText(center + ImVec2(0, 30) - textSize / 2, PID_COLOR_OUTLINE, label);
}

void DrawCheckValve(ImVec2 center, char orientation) {
  ImDrawList *dl = ImGui::GetWindowDrawList();

  ImVec2 tl, bl, tr, br;

  if (orientation == 'H') {
    // true coordinates
    tl = center + ImVec2(-CHECK_VALVE_SIZE, -CHECK_VALVE_SIZE * 0.5);
    bl = center + ImVec2(-CHECK_VALVE_SIZE, CHECK_VALVE_SIZE * 0.5);
    tr = center + ImVec2(CHECK_VALVE_SIZE, -CHECK_VALVE_SIZE * 0.5);
    br = center + ImVec2(CHECK_VALVE_SIZE, CHECK_VALVE_SIZE * 0.5);
  } else {
    // rotate coords
    tl = center + ImVec2(-CHECK_VALVE_SIZE * 0.5, CHECK_VALVE_SIZE);  // bl
    bl = center + ImVec2(CHECK_VALVE_SIZE * 0.5, CHECK_VALVE_SIZE);   // br
    tr = center + ImVec2(-CHECK_VALVE_SIZE * 0.5, -CHECK_VALVE_SIZE); // tl
    br = center + ImVec2(CHECK_VALVE_SIZE * 0.5, -CHECK_VALVE_SIZE);  // tr
  }

  ImColor line_color = PID_COLOR_OUTLINE;
  int edge_thk = 2;

  dl->AddLine(bl, tl, line_color, edge_thk);
  dl->AddLine(tl, br, line_color, edge_thk);
  dl->AddLine(br, tr, line_color, edge_thk);

  float arrow_offset_hor = 10;
  float arrow_offset_vert = 5;
  if (orientation == 'H') {
    dl->AddLine(bl + ImVec2(arrow_offset_hor, arrow_offset_vert), br + ImVec2(-arrow_offset_hor, arrow_offset_vert), line_color, edge_thk);
    dl->AddTriangle(br + ImVec2(-arrow_offset_hor, arrow_offset_vert - 4), br + ImVec2(-arrow_offset_hor, arrow_offset_vert + 4), br + ImVec2(-arrow_offset_hor + 5, arrow_offset_vert), line_color);
    dl->AddTriangleFilled(br + ImVec2(-arrow_offset_hor, arrow_offset_vert - 4), br + ImVec2(-arrow_offset_hor, arrow_offset_vert + 4), br + ImVec2(-arrow_offset_hor + 5, arrow_offset_vert),
                          line_color);

  } else {
    dl->AddLine(tl + ImVec2(-arrow_offset_vert, -arrow_offset_hor), tr + ImVec2(-arrow_offset_vert, arrow_offset_hor), line_color, edge_thk);
    dl->AddTriangle(tl + ImVec2(-arrow_offset_vert - 4, -arrow_offset_hor), tl + ImVec2(-arrow_offset_vert + 4, -arrow_offset_hor), tl + ImVec2(-arrow_offset_vert, -arrow_offset_hor + 5), line_color);
    dl->AddTriangleFilled(tl + ImVec2(-arrow_offset_vert - 4, -arrow_offset_hor), tl + ImVec2(-arrow_offset_vert + 4, -arrow_offset_hor), tl + ImVec2(-arrow_offset_vert, -arrow_offset_hor + 5),
                          line_color);
  }
}

// TODO - this should support vertical as well I was just lazy
void DrawManualValve(ImVec2 center, const char *label) {
  ImDrawList *dl = ImGui::GetWindowDrawList();

  ImVec2 tl = center + ImVec2(-REG_SIZE, -REG_SIZE * 0.5);
  ImVec2 bl = center + ImVec2(-REG_SIZE, REG_SIZE * 0.5);
  ImVec2 tr = center + ImVec2(REG_SIZE, -REG_SIZE * 0.5);
  ImVec2 br = center + ImVec2(REG_SIZE, REG_SIZE * 0.5);

  dl->AddTriangleFilled(center, bl, tl, PID_COLOR_OUTLINE);
  dl->AddTriangleFilled(center, tr, br, PID_COLOR_OUTLINE);
  dl->AddCircleFilled(center, REG_SIZE * 0.4, PID_COLOR_OUTLINE);

  // manual handle
  const int handle_height = 16;
  dl->AddLine(center, center - ImVec2(0, handle_height), PID_COLOR_OUTLINE, 2);
  dl->AddLine(center - ImVec2(7, handle_height), center - ImVec2(-7, handle_height), PID_COLOR_OUTLINE, 2);

  ImVec2 textSize = ImGui::CalcTextSize(label);
  dl->AddText(center + ImVec2(0, 30) - textSize / 2, PID_COLOR_OUTLINE, label);
}

ImVec2 SampleCubicBezier(ImVec2 p0, ImVec2 p1, ImVec2 p2, ImVec2 p3, float t) {
  float u = 1.0 - t;
  return p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t);
}

// Helper to interpolate the crossing point between two sampled bezier points
ImVec2 CrossingPoint(ImVec2 pPrev, ImVec2 p, float target_y) {
  float alpha = (target_y - pPrev.y) / (p.y - pPrev.y);
  return pPrev + (p - pPrev) * alpha;
}

void DrawTank(ImVec2 center, const char *label, ImColor col, float fillLevel) {
  ImDrawList *dl = ImGui::GetWindowDrawList();
  ImVec2 tl = center + ImVec2(-TANK_WIDTH * 0.5, -TANK_HEIGHT * 0.5);
  ImVec2 bl = center + ImVec2(-TANK_WIDTH * 0.5, TANK_HEIGHT * 0.5);
  ImVec2 tr = center + ImVec2(TANK_WIDTH * 0.5, -TANK_HEIGHT * 0.5);
  ImVec2 br = center + ImVec2(TANK_WIDTH * 0.5, TANK_HEIGHT * 0.5);

  const int BEZIER_SEGS = 16;

  ImVec2 bc_p0 = bl;
  ImVec2 bc_p1 = bl + ImVec2(TANK_WIDTH * 0.2, TANK_WIDTH * 0.4);
  ImVec2 bc_p2 = br + ImVec2(-TANK_WIDTH * 0.2, TANK_WIDTH * 0.4);
  ImVec2 bc_p3 = br;

  ImVec2 tc_p0 = tl;
  ImVec2 tc_p1 = tl + ImVec2(TANK_WIDTH * 0.2, -TANK_WIDTH * 0.4);
  ImVec2 tc_p2 = tr + ImVec2(-TANK_WIDTH * 0.2, -TANK_WIDTH * 0.4);
  ImVec2 tc_p3 = tr;

  float capHeight = SampleCubicBezier(bc_p0, bc_p1, bc_p2, bc_p3, 0.5).y - bc_p0.y;
  float totalHeight = TANK_HEIGHT + capHeight * 2.0f;
  float fillY = bl.y + capHeight - fillLevel * totalHeight;

  dl->AddLine(center + ImVec2(5, totalHeight / 2), ImVec2(center.x + 5, fillY), col, 4);

  // TODO - probably some room for optimizations / cleanup here
  bool started = false;
  for (int i = 0; i <= BEZIER_SEGS; i++) {
    float t = (float)i / BEZIER_SEGS;
    ImVec2 p = SampleCubicBezier(bc_p0, bc_p1, bc_p2, bc_p3, t);
    if (p.y >= fillY) {
      if (!started) {
        if (i > 0) {
          ImVec2 pPrev = SampleCubicBezier(bc_p0, bc_p1, bc_p2, bc_p3, (float)(i - 1) / BEZIER_SEGS);
          dl->PathLineTo(CrossingPoint(pPrev, p, fillY));
        }
        started = true;
      }
      dl->PathLineTo(p);
    } else if (started) {
      ImVec2 pPrev = SampleCubicBezier(bc_p0, bc_p1, bc_p2, bc_p3, (float)(i - 1) / BEZIER_SEGS);
      dl->PathLineTo(CrossingPoint(pPrev, p, fillY));
      break;
    }
  }
  dl->PathFillConvex(col);

  if (fillY < bl.y) {
    dl->PathLineTo(ImVec2(br.x, br.y));
    dl->PathLineTo(ImVec2(br.x, std::max(fillY, tr.y)));
    dl->PathLineTo(ImVec2(bl.x, std::max(fillY, tl.y)));
    dl->PathLineTo(ImVec2(bl.x, bl.y));
    dl->PathFillConvex(col);
  }

  if (fillY < tl.y) {
    bool ended = false;
    ImVec2 pPrev = tc_p0;

    for (int i = 0; i <= BEZIER_SEGS; i++) {
      float t = (float)i / BEZIER_SEGS;
      ImVec2 p = SampleCubicBezier(tc_p0, tc_p1, tc_p2, tc_p3, t);

      if (p.y >= fillY) { // should include point
        if (!ended) {
          dl->PathLineTo(p);
        } else {
          dl->PathLineTo(CrossingPoint(pPrev, p, fillY));
          ended = false;
        }
      } else {
        if (!ended) {
          dl->PathLineTo(CrossingPoint(pPrev, p, fillY));
          ended = true;
        }
      }

      pPrev = p;
    }

    dl->PathLineTo(tc_p3);
    dl->PathFillConvex(col);
  }

  // Outline
  dl->AddLine(tl, bl, PID_COLOR_OUTLINE, 2);
  dl->AddLine(tr, br, PID_COLOR_OUTLINE, 2);
  dl->AddBezierCubic(tc_p0, tc_p1, tc_p2, tc_p3, PID_COLOR_OUTLINE, 2);
  dl->AddBezierCubic(bc_p0, bc_p1, bc_p2, bc_p3, PID_COLOR_OUTLINE, 2);

  ImVec2 textSize = ImGui::CalcTextSize(label);

  ImColor bk_color = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];

  dl->AddRectFilled(center - textSize / 2 + ImVec2(-5, -55), center + textSize / 2 + ImVec2(5, -45), bk_color, 5);
  dl->AddRect(center - textSize / 2 + ImVec2(-5, -55), center + textSize / 2 + ImVec2(5, -45), PID_COLOR_OUTLINE, 5);
  dl->AddText(center - textSize / 2 + ImVec2(0, -50), PID_COLOR_OUTLINE, label);
}

void DrawLargeNozzle(ImVec2 center) {
  ImDrawList *dl = ImGui::GetWindowDrawList();
  ImVec2 tl = center + ImVec2(-LARGE_NOZZEL_WIDTH * 0.5, -LARGE_NOZZEL_HEIGHT * 0.5);
  ImVec2 ml = center + ImVec2(-LARGE_NOZZEL_WIDTH * 0.5, LARGE_NOZZEL_HEIGHT * 0.1);
  ImVec2 bl = center + ImVec2(-LARGE_NOZZEL_WIDTH * 0.4, LARGE_NOZZEL_HEIGHT * 0.5);
  ImVec2 bezier_l = center + ImVec2(-LARGE_NOZZEL_WIDTH * 0.1, LARGE_NOZZEL_HEIGHT * 0.2);

  ImVec2 tr = center + ImVec2(LARGE_NOZZEL_WIDTH * 0.5, -LARGE_NOZZEL_HEIGHT * 0.5);
  ImVec2 mr = center + ImVec2(LARGE_NOZZEL_WIDTH * 0.5, LARGE_NOZZEL_HEIGHT * 0.1);
  ImVec2 br = center + ImVec2(LARGE_NOZZEL_WIDTH * 0.4, LARGE_NOZZEL_HEIGHT * 0.5);
  ImVec2 bezier_r = center + ImVec2(LARGE_NOZZEL_WIDTH * 0.1, LARGE_NOZZEL_HEIGHT * 0.2);

  dl->AddLine(tl, tr, PID_COLOR_OUTLINE);
  dl->AddLine(ml, tl, PID_COLOR_OUTLINE);
  dl->AddLine(mr, tr, PID_COLOR_OUTLINE);
  dl->AddBezierQuadratic(ml, bezier_l, bl, PID_COLOR_OUTLINE, 1);
  dl->AddBezierQuadratic(mr, bezier_r, br, PID_COLOR_OUTLINE, 1);

  ImVec2 text_size = ImGui::CalcTextSize("OX Inj");
  dl->AddText(center + ImVec2(0, -LARGE_NOZZEL_HEIGHT * 0.5 + 25) - text_size / 2, PID_COLOR_OUTLINE, "OX Inj");
  dl->AddText(center + ImVec2(0, -LARGE_NOZZEL_HEIGHT * 0.5 + 75) - text_size / 2, PID_COLOR_OUTLINE, "FU Inj");
  dl->AddLine(tl + ImVec2(0, 50), tr + ImVec2(0, 50), PID_COLOR_OUTLINE);
  dl->AddLine(tl + ImVec2(0, 100), tr + ImVec2(0, 100), PID_COLOR_OUTLINE);
}

void DrawSmallNozzle(ImVec2 center, char orientation) {
  ImDrawList *dl = ImGui::GetWindowDrawList();

  int flip;
  if (orientation == 'R') {
    flip = 1;
  } else {
    flip = -1;
  }

  ImVec2 tl = center + ImVec2(-SMALL_NOZZEL_LENGTH * 0.5 * flip, -SMALL_NOZZEL_WIDTH * 0.5);
  ImVec2 tm = center + ImVec2(SMALL_NOZZEL_LENGTH * 0.1 * flip, -SMALL_NOZZEL_WIDTH * 0.5);

  ImVec2 bl = center + ImVec2(-SMALL_NOZZEL_LENGTH * 0.5 * flip, SMALL_NOZZEL_WIDTH * 0.5);
  ImVec2 bm = center + ImVec2(SMALL_NOZZEL_LENGTH * 0.1 * flip, SMALL_NOZZEL_WIDTH * 0.5);

  ImVec2 bezier_pt = center + ImVec2(SMALL_NOZZEL_LENGTH * 0.5 * flip, 0);

  ImVec2 tri_1 = center + ImVec2(SMALL_NOZZEL_LENGTH * 0.3 * flip, 0);
  ImVec2 tri_2 = center + ImVec2(SMALL_NOZZEL_LENGTH * 0.5 * flip, -SMALL_NOZZEL_WIDTH * 0.25);
  ImVec2 tri_3 = center + ImVec2(SMALL_NOZZEL_LENGTH * 0.5 * flip, SMALL_NOZZEL_WIDTH * 0.25);

  dl->AddLine(tl, bl, PID_COLOR_OUTLINE);
  dl->AddLine(tm, tl, PID_COLOR_OUTLINE);
  dl->AddLine(bm, bl, PID_COLOR_OUTLINE);
  dl->AddBezierQuadratic(tm, bezier_pt, bm, PID_COLOR_OUTLINE, 1);
  dl->AddTriangle(tri_1, tri_2, tri_3, PID_COLOR_OUTLINE, 1);
}

void DrawIgniter(ImVec2 center) {
  ImDrawList *dl = ImGui::GetWindowDrawList();

  ImVec2 tl = center + ImVec2(-SMALL_NOZZEL_WIDTH * 0.5, -SMALL_NOZZEL_LENGTH * 0.5);
  ImVec2 ml = center + ImVec2(-SMALL_NOZZEL_WIDTH * 0.5, SMALL_NOZZEL_LENGTH * 0.1);

  ImVec2 tr = center + ImVec2(SMALL_NOZZEL_WIDTH * 0.5, -SMALL_NOZZEL_LENGTH * 0.5);
  ImVec2 mr = center + ImVec2(SMALL_NOZZEL_WIDTH * 0.5, SMALL_NOZZEL_LENGTH * 0.1);

  ImVec2 bezier_pt = center + ImVec2(0, SMALL_NOZZEL_LENGTH * 0.5);

  ImVec2 tri_1 = center + ImVec2(0, SMALL_NOZZEL_LENGTH * 0.3);
  ImVec2 tri_2 = center + ImVec2(-SMALL_NOZZEL_WIDTH * 0.25, SMALL_NOZZEL_LENGTH * 0.5);
  ImVec2 tri_3 = center + ImVec2(SMALL_NOZZEL_WIDTH * 0.25, SMALL_NOZZEL_LENGTH * 0.5);

  dl->AddLine(tl, tr, PID_COLOR_OUTLINE);
  dl->AddLine(ml, tl, PID_COLOR_OUTLINE);
  dl->AddLine(mr, tr, PID_COLOR_OUTLINE);
  dl->AddBezierQuadratic(ml, bezier_pt, mr, PID_COLOR_OUTLINE, 1);
  dl->AddTriangle(tri_1, tri_2, tri_3, PID_COLOR_OUTLINE, 1);
}

void DrawReadout(ImVec2 center, ImVec2 attach_point, const char *name, const char *unit, float reading, char attach_direction) {
  ImDrawList *dl = ImGui::GetWindowDrawList();

  char text_buf[20];
  snprintf(text_buf, sizeof(text_buf), "%.1lf", reading);

  ImVec2 text_size = ImGui::CalcTextSize(text_buf);
  ImVec2 max_text_size = ImGui::CalcTextSize("8888.8");
  ImVec2 unit_label_size = ImGui::CalcTextSize(unit);
  ImVec2 pid_label_size = ImGui::CalcTextSize(name);

  ImVec2 rect_tl = center - max_text_size / 2 - ImVec2(5, 5) - ImVec2(max_text_size.x, 0) * 0.35;
  ImVec2 rect_br = center + max_text_size / 2 + ImVec2(5, 5) - ImVec2(max_text_size.x, 0) * 0.35;

  ImVec2 rect_tm = center - ImVec2(0, max_text_size.y / 2) - ImVec2(0, 5) - ImVec2(max_text_size.x, 0) * 0.35;
  ImVec2 rect_bm = center + ImVec2(0, max_text_size.y / 2) + ImVec2(0, 5) - ImVec2(max_text_size.x, 0) * 0.35;
  ImVec2 rect_lm = center - ImVec2(max_text_size.x / 2, 0) - ImVec2(5, 0) - ImVec2(max_text_size.x, 0) * 0.35;

  dl->AddRectFilled(rect_tl, rect_br, PID_COLOR_READOUT_BOX, 5);
  dl->AddRect(rect_tl, rect_br, PID_COLOR_OUTLINE, 5);

  dl->AddText(center - text_size / 2 - ImVec2(max_text_size.x, 0) * 0.35, PID_COLOR_READOUT_BOX_TEXT, text_buf);
  dl->AddText(center - ImVec2(0, unit_label_size.y / 2) + ImVec2(max_text_size.x, 0) * 0.35, PID_COLOR_OUTLINE, unit);

  int flip = 1;
  if (attach_direction == 'b') {
    flip = -1;
  }
  dl->AddText(center - pid_label_size / 2 + ImVec2(0, 30) * flip - ImVec2(max_text_size.x, 0) * 0.35, PID_COLOR_OUTLINE, name);

  if (attach_direction == 't') {
    dl->AddLine(rect_tm, rect_tm + ImVec2(0, -15), PID_COLOR_OUTLINE, 1);
    dl->AddLine(rect_tm + ImVec2(0, -15), ImVec2(attach_point.x, rect_tm.y - 15), PID_COLOR_OUTLINE, 1);
  } else if (attach_direction == 'b') {
    dl->AddLine(rect_bm, rect_bm + ImVec2(0, 15), PID_COLOR_OUTLINE, 1);
    dl->AddLine(rect_bm + ImVec2(0, 15), ImVec2(attach_point.x, rect_bm.y + 15), PID_COLOR_OUTLINE, 1);
  } else if (attach_direction == 'l') {
    dl->AddLine(rect_lm, attach_point, PID_COLOR_OUTLINE, 1);
  } else if (attach_direction == 'u') {
    dl->AddLine(rect_tm, ImVec2(rect_tm.x, attach_point.y), PID_COLOR_OUTLINE, 1);
  }
}

bool MouseInValveHitbox(ImVec2 center, char orientation) {
  ImVec2 tl, br; // define key points of the triangles

  if (orientation == 'H') {
    // true coordinates
    tl = center + ImVec2(-VALVE_SIZE, -VALVE_SIZE * 0.5);
    br = center + ImVec2(VALVE_SIZE, VALVE_SIZE * 0.5);
  } else {
    // rotate coords
    tl = center + ImVec2(-VALVE_SIZE * 0.5, -VALVE_SIZE);
    br = center + ImVec2(VALVE_SIZE * 0.5, VALVE_SIZE);
  }

  ImVec2 mouse_pos = ImGui::GetMousePos();
  return (mouse_pos.x >= tl.x && mouse_pos.x <= br.x && mouse_pos.y >= tl.y && mouse_pos.y <= br.y);
}
