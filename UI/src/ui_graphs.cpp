#include "ui_graphs.h"
#include "implot.h"
#include "implot3d.h"
#include "ui_components.h"

ImVec4 axis_colors[3] = {{1.0, 0.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, {0.0, 0.0, 1.0, 1.0}};

void scrolling_line_chart(scrolling_line_chart_arg_t arg, float y1[FLIGHT_HISTORY_LENGTH], float y2[FLIGHT_HISTORY_LENGTH], float y3[FLIGHT_HISTORY_LENGTH]) {
  centered_text(arg.plot_title);
  if (ImPlot::BeginPlot(arg.render_title, ImVec2(-1, 175))) { // width = fill, height auto
    ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1000, ImPlotCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, arg.y_min, arg.y_max);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest);

    ImPlotSpec spec;
    spec.LineColor = axis_colors[0];
    ImPlot::PlotLine(arg.y1_label, &y1[FlightHistory.read_start_pos], FLIGHT_HISTORY_LENGTH, 1, 0, spec);
    spec.LineColor = axis_colors[1];
    ImPlot::PlotLine(arg.y2_label, &y2[FlightHistory.read_start_pos], FLIGHT_HISTORY_LENGTH, 1, 0, spec);
    spec.LineColor = axis_colors[2];
    ImPlot::PlotLine(arg.y3_label, &y3[FlightHistory.read_start_pos], FLIGHT_HISTORY_LENGTH, 1, 0, spec);
    ImPlot::EndPlot();
  }
}

#define NUM_PTS 17
#define NUM_EDG 23
ImVec4 cube_verts[NUM_PTS] = {{-0.75, -0.75, -1.5, 0},
                              {0.75, -0.75, -1.5, 0},
                              {0.75, 0.75, -1.5, 0},
                              {-0.75, 0.75, -1.5, 0},
                              {-0.75, -0.75, 1.75, 0},
                              {0.75, -0.75, 1.75, 0},
                              {0.75, 0.75, 1.75, 0},
                              {-0.75, 0.75, 1.75, 0},
                              {0, 0, 0, 0},
                              {3, 0, 0, 0},
                              {0, 3, 0, 0},
                              {0, 0, 3, 0},
                              {-1.25, -1.25, -2.5, 0},
                              {1.25, -1.25, -2.5, 0},
                              {1.25, 1.25, -2.5, 0},
                              {-1.25, 1.25, -2.5, 0},
                              {0, 0, 2.5, 0}};
int cube_edges[NUM_EDG][2] = {{0, 1}, {1, 2},  {2, 3},  {3, 0},  {4, 5},  {5, 6},  {6, 7},  {7, 4},  {0, 4},  {1, 5},  {2, 6}, {3, 7},
                              {8, 9}, {8, 10}, {8, 11}, {0, 12}, {1, 13}, {2, 14}, {3, 15}, {4, 16}, {5, 16}, {6, 16}, {7, 16}};

ImVec4 quatRot(ImVec4 q, ImVec4 vtx) {
  ImVec4 out;
  out.x = vtx.x * (1 - 2 * (q.y * q.y + q.z * q.z)) + vtx.y * (2 * (q.x * q.y - q.w * q.z)) + vtx.z * (2 * (q.x * q.z + q.w * q.y));
  out.y = vtx.x * (2 * (q.x * q.y + q.w * q.z)) + vtx.y * (1 - 2 * (q.x * q.x + q.z * q.z)) + vtx.z * (2 * (q.y * q.z - q.w * q.x));
  out.z = vtx.x * (2 * (q.x * q.z - q.w * q.y)) + vtx.y * (2 * (q.y * q.z + q.w * q.x)) + vtx.z * (1 - 2 * (q.x * q.x + q.y * q.y));
  return out;
}

void rotatable_cube_plot(ImVec4 q) {
  // rotate cube vertices
  ImVec4 rot[NUM_PTS];
  for (int i = 0; i < NUM_PTS; i++) {
    rot[i] = quatRot(q, cube_verts[i]);
  }

  if (ImPlot3D::BeginPlot("##Cube3D", ImVec2(-1, 500))) {
    ImPlot3D::SetupAxes("East (m)", "North (m)", "Up (m)");
    ImPlot3D::SetupAxesLimits(-3, 3, -3, 3, -3, 3, ImPlot3DCond_Always);

    // plot cube
    for (int i = 0; i < 12; i++) {
      ImVec4 a = rot[cube_edges[i][0]];
      ImVec4 b = rot[cube_edges[i][1]];
      // Draw line segment
      double xs[2] = {-a.y, -b.y};
      double ys[2] = {a.x, b.x};
      double zs[2] = {a.z, b.z};
      ImPlot3D::PlotLine("##edge", xs, ys, zs, 2);
    }
    for (int i = 15; i < NUM_EDG; i++) {
      ImVec4 a = rot[cube_edges[i][0]];
      ImVec4 b = rot[cube_edges[i][1]];
      // Draw line segment
      double xs[2] = {-a.y, -b.y};
      double ys[2] = {a.x, b.x};
      double zs[2] = {a.z, b.z};
      ImPlot3D::PlotLine("##edge", xs, ys, zs, 2);
    }

    const char *axis_labels[3] = {"yaw", "pitch", "roll"};

    // plot axis
    for (int i = 0; i < 3; i++) {
      ImVec4 a = rot[cube_edges[i + 12][0]];
      ImVec4 b = rot[cube_edges[i + 12][1]];
      // Draw line segment
      double xs[2] = {-a.y, -b.y};
      double ys[2] = {a.x, b.x};
      double zs[2] = {a.z, b.z};
      ImPlot3DSpec spec;
      spec.LineColor = axis_colors[i];
      ImPlot3D::PlotLine(axis_labels[i], xs, ys, zs, 2, spec);
    }

    ImPlot3D::EndPlot();
  }
}