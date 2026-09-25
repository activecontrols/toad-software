#pragma once

#include "flight_history.h"
#include "implot.h"
#include "implot3d.h"

typedef struct {
  const char *render_title;
  const char *plot_title;
  const char *y1_label;
  const char *y2_label;
  const char *y3_label;
  double y_max;
  double y_min;
} scrolling_line_chart_arg_t;

void scrolling_line_chart(scrolling_line_chart_arg_t arg, float y1[FLIGHT_HISTORY_LENGTH * 2],
                          float y2[FLIGHT_HISTORY_LENGTH * 2], float y3[FLIGHT_HISTORY_LENGTH * 2]);
void rotatable_cube_plot(ImVec4 q);
