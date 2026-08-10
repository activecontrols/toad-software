#include "imgui_internal.h"
#include "math.h"
#include "ui.h"
#include "ui_components.h"
#include "ui_graphs.h"

#define IMU_ACCEL_PANEL "imu_accel_panel"
#define IMU_GYRO_PANEL "imu_gyro_panel"
#define MAG_PANEL "mag_panel"
#define GYRO_BIAS_PANEL "gyro_bias_panel"
#define MAG_BIAS_PANEL "mag_bias_panel"
#define ACCEL_BIAS_PANEL "accel_bias_panel"
#define GPS_POS_PANEL "gps_pos_panel"
#define GPS_VEL_PANEL "gps_vel_panel"
#define GPS_VERT_PANEL "gps_vert_panel"
#define EST_POS_PANEL "est_pos_panel"
#define EST_ROT_PANEL "est_rot_panel"
#define CONTROLLER_OUTPUT_PANEL "controller_output_panel"
#define GIMBAL_OUTPUT_PANEL "gimbal_output_panel"

void imu_accel_panel() {
  ImGui::Begin(IMU_ACCEL_PANEL);

  scrolling_line_chart_arg_t imu_acc;
  imu_acc.plot_title = "IMU Accel";
  imu_acc.render_title = "##IMU Accel";
  imu_acc.y1_label = "x";
  imu_acc.y2_label = "y";
  imu_acc.y3_label = "z";
  imu_acc.y_max = 15;
  imu_acc.y_min = -5;

  scrolling_line_chart(imu_acc, FlightHistory.accel_x, FlightHistory.accel_y, FlightHistory.accel_z);

  ImGui::End();
}

void imu_gyro_panel() {
  ImGui::Begin(IMU_GYRO_PANEL);

  scrolling_line_chart_arg_t imu_gyro;
  imu_gyro.plot_title = "IMU Gyro";
  imu_gyro.render_title = "##IMU Gyro";
  imu_gyro.y1_label = "yaw";
  imu_gyro.y2_label = "pitch";
  imu_gyro.y3_label = "roll";
  imu_gyro.y_max = 0.5;
  imu_gyro.y_min = -0.5;

  scrolling_line_chart(imu_gyro, FlightHistory.gyro_yaw, FlightHistory.gyro_pitch, FlightHistory.gyro_roll);

  ImGui::End();
}

void mag_panel() {
  ImGui::Begin(MAG_PANEL);

  scrolling_line_chart_arg_t mag;
  mag.plot_title = "Mag";
  mag.render_title = "##Mag";
  mag.y1_label = "x";
  mag.y2_label = "y";
  mag.y3_label = "z";
  mag.y_max = 1.5;
  mag.y_min = -1.5;
  scrolling_line_chart(mag, FlightHistory.mag_x, FlightHistory.mag_y, FlightHistory.mag_z);

  ImGui::End();
}

void gyro_bias_panel() {
  ImGui::Begin(GYRO_BIAS_PANEL);

  scrolling_line_chart_arg_t mag;
  mag.plot_title = "Gyro Bias";
  mag.render_title = "##Mag";
  mag.y1_label = "bias pitch";
  mag.y2_label = "bias yaw";
  mag.y3_label = "bias roll";
  mag.y_max = 0.1;
  mag.y_min = -0.1;
  scrolling_line_chart(mag, FlightHistory.gyro_bias_pitch, FlightHistory.gyro_bias_yaw, FlightHistory.gyro_bias_roll);

  ImGui::End();
}

void accel_bias_panel() {
  ImGui::Begin(ACCEL_BIAS_PANEL);

  scrolling_line_chart_arg_t mag;
  mag.plot_title = "Accel Bias";
  mag.render_title = "##Mag";
  mag.y1_label = "bias x";
  mag.y2_label = "bias y";
  mag.y3_label = "bias z";
  mag.y_max = 0.1;
  mag.y_min = -0.1;
  scrolling_line_chart(mag, FlightHistory.accel_bias_x, FlightHistory.accel_bias_y, FlightHistory.accel_bias_z);

  ImGui::End();
}

void mag_bias_panel() {
  ImGui::Begin(MAG_BIAS_PANEL);

  scrolling_line_chart_arg_t mag;
  mag.plot_title = "Mag Bias";
  mag.render_title = "##Mag";
  mag.y1_label = "bias x";
  mag.y2_label = "bias y";
  mag.y3_label = "bias z";
  mag.y_max = 0.5;
  mag.y_min = -0.5;
  scrolling_line_chart(mag, FlightHistory.mag_bias_x, FlightHistory.mag_bias_y, FlightHistory.mag_bias_z);

  ImGui::End();
}

void gps_pos_panel() {
  ImGui::Begin(GPS_POS_PANEL);

  float state_x = -FlightHistory.gps_pos_west;
  float state_y = FlightHistory.gps_pos_north;
  float target_x = -FlightHistory.target_pos_west;
  float target_y = FlightHistory.target_pos_north;

  centered_text("GPS Position");
  if (ImPlot::BeginPlot("##GPS Position", ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("East (m)", "North (m)");
    ImPlot::SetupAxesLimits(-2, 2, -1, 1);
    ImPlot::PlotScatter("State", &state_x, &state_y, 1);
    ImPlot::PlotScatter("Target", &target_x, &target_y, 1);

    const int N = 64;
    double xs[N];
    double ys[N];

    double r = FlightHistory.gps_hor_prec;

    for (int i = 0; i < N; ++i) {
      double theta = 2.0 * 3.1415 * i / (N - 1);
      xs[i] = state_x + r * cos(theta);
      ys[i] = state_y + r * sin(theta);
    }
    ImPlotSpec spec;
    spec.LineColor = ImPlot3D::GetColormapColor(0, ImPlot3DColormap_Deep);
    ImPlot::PlotLine("##Accuracy", xs, ys, N, spec);

    ImPlot::EndPlot();
  }

  ImGui::End();
}

void gps_vel_panel() {
  ImGui::Begin(GPS_VEL_PANEL);

  centered_text("GPS Velocity");
  if (ImPlot::BeginPlot("##GPS Velocity", ImVec2(-1, 200), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("East (m/s)", "North (m/s)");
    ImPlot::SetupAxesLimits(-5, 5, -5, 5);

    double gps_x[2] = {0, -FlightHistory.gps_vel_west};
    double gps_y[2] = {0, FlightHistory.gps_vel_north};
    ImPlotSpec spec;
    spec.LineWeight = 4;
    ImPlot::PlotLine("##GPS Velocity", gps_x, gps_y, 2, spec);

    ImPlot::EndPlot();
  }

  ImGui::End();
}

void gps_vert_panel() {
  ImGui::Begin(GPS_VERT_PANEL);

  centered_text("Altitude");
  ImGui::Text("     GPS: %5.2f m", FlightHistory.gps_pos_up);
  ImGui::Text("  Target: %5.2f m", FlightHistory.target_pos_up);
  ImGui::Dummy(ImVec2(0, 50)); // Add vertical spacing
  centered_text("Vert Velocity");
  ImGui::Text("     GPS: %5.2f m/s", FlightHistory.gps_vel_up);

  ImGui::End();
}

void estimated_pos_panel() {
  ImGui::Begin(EST_POS_PANEL);

  centered_text("Estimated Pos");
  if (ImPlot3D::BeginPlot("##Estimated Pos", ImVec2(-1, 500))) {
    double cs_x[2] = {-FlightHistory.state_pos_west, -FlightHistory.state_pos_west};
    double cs_y[2] = {FlightHistory.state_pos_north, FlightHistory.state_pos_north};
    double cs_z[2] = {0, FlightHistory.state_pos_up};

    double target_x[2] = {-FlightHistory.target_pos_west, -FlightHistory.target_pos_west};
    double target_y[2] = {FlightHistory.target_pos_north, FlightHistory.target_pos_north};
    double target_z[2] = {0, FlightHistory.target_pos_up};

    ImPlot3D::SetupAxes("East (m)", "North (m)", "Up (m)");
    ImPlot3D::SetupAxisLimits(ImAxis3D_Z, 0, 2.5);
    ImPlot3D::SetupAxisLimits(ImAxis3D_X, -1, 1);
    ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -1, 1);

    ImPlot3DSpec marker_spec;
    marker_spec.Marker = ImPlot3DMarker_Circle;
    marker_spec.MarkerSize = 5;
    marker_spec.MarkerFillColor = ImPlot3D::GetColormapColor(0, ImPlot3DColormap_Deep);
    ImPlot3D::PlotScatter("State", cs_x, cs_y, cs_z, 2, marker_spec);

    marker_spec.Marker = ImPlot3DMarker_Diamond;
    marker_spec.MarkerSize = 5;
    marker_spec.MarkerFillColor = ImPlot3D::GetColormapColor(1, ImPlot3DColormap_Deep);
    ImPlot3D::PlotScatter("Target", target_x, target_y, target_z, 2, marker_spec);

    ImPlot3DSpec line_spec;
    line_spec.LineColor = ImPlot3D::GetColormapColor(0, ImPlot3DColormap_Deep);
    line_spec.LineWeight = 2;
    ImPlot3D::PlotLine("State", cs_x, cs_y, cs_z, 2, line_spec);

    line_spec.LineColor = ImPlot3D::GetColormapColor(1, ImPlot3DColormap_Deep);
    line_spec.LineWeight = 2;
    ImPlot3D::PlotLine("Target", target_x, target_y, target_z, 2, line_spec);

    ImPlot3D::EndPlot();
  }

  ImGui::End();
}

void estimated_orientation_panel() {
  ImGui::Begin(EST_ROT_PANEL);

  centered_text("Orientation");
  ImVec4 q;
  q.x = FlightHistory.state_q_vec_0;
  q.y = FlightHistory.state_q_vec_1;
  q.z = FlightHistory.state_q_vec_2;
  q.w = FlightHistory.state_q_vec_new;
  rotatable_cube_plot(q);

  ImGui::End();
}

void controller_output_panel() {
  ImGui::Begin(CONTROLLER_OUTPUT_PANEL);

  centered_text("Controller Output");
  ImGui::Text("  Target Thrust: %5.2f N", FlightHistory.thrust_N);
  ImGui::Text("    Target Roll: %5.2f rad/s^2", FlightHistory.roll_rad_sec_squared);
  ImGui::Text("         Thrust: %5.2f %%", FlightHistory.thrust_perc);
  ImGui::Text("   Differential: %5.2f %%", FlightHistory.diffy_perc);

  ImGui::Dummy(ImVec2(0, 100));

  ImGui::Text("Elasped Time: %5.2f s", FlightHistory.elapsed_time);
  colored_flag("    GND Flag", FlightHistory.GND_flag, ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f), ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f), "##gnd_flag");
  ImGui::TableSetColumnIndex(1);
  if (FlightHistory.flight_armed) {
    colored_flag("       Armed", FlightHistory.flight_armed, ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f), ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f), "##armed_flag");
  } else {
    colored_flag("     Not Armed", FlightHistory.flight_armed, ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f), ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f), "##armed_flag");
  }

  if (FlightHistory.rtk_status == 0) {
    colored_flag("     NO RTK", 1, ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f), ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f), "##rtk_flag");
  } else if (FlightHistory.rtk_status == 1) {
    colored_flag(" RTK FLOAT", 1, ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f), ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f), "##rtk_flag");
  } else if (FlightHistory.rtk_status == 2) {
    colored_flag(" RTK FIX", 0, ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f), ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f), "##rtk_flag");
  }

  ImGui::Text("GPS Horizontal Precision: %5.2f cm", FlightHistory.gps_hor_prec * 100);
  ImGui::Text("GPS Vertical Precision: %5.2f cm", FlightHistory.gps_ver_prec * 100);
  ImGui::Text("GPS Sat Count: %d", FlightHistory.gps_sat_count);

  ImGui::End();
}

void gimbal_output_panel() {
  ImGui::Begin(GIMBAL_OUTPUT_PANEL);

  float x = FlightHistory.gimbal_yaw_raw;
  float y = FlightHistory.gimbal_pitch_raw;

  centered_text("Gimbal Command");
  if (ImPlot::BeginPlot("##Gimbal Command", ImVec2(-1, 250), ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("Yaw (deg)", "Pitch (deg)");
    ImPlot::SetupAxesLimits(-15, 15, -15, 15, ImPlotCond_Always);
    ImPlot::PlotScatter("##Gimbal", &x, &y, 1);
    ImPlot::EndPlot();
  }

  ImGui::End();
}

void build_gnc_dock_layout(ImGuiID dockspace_id) {
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

  ImGuiID left_panel = dockspace_id;
  ImGuiID right_panel = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, 0.5, nullptr, &left_panel);

  ImGuiID live_sensor_panel = left_panel;

  ImGuiID live_sensor_top = live_sensor_panel;
  ImGuiID live_sensor_middle = ImGui::DockBuilderSplitNode(live_sensor_panel, ImGuiDir_Down, 2.0 / 3.0, nullptr, &live_sensor_top);
  ImGuiID live_sensor_bottom = ImGui::DockBuilderSplitNode(live_sensor_middle, ImGuiDir_Down, 0.5, nullptr, &live_sensor_middle);

  ImGuiID live_sensor_tl = live_sensor_top;
  ImGuiID live_sensor_tr = ImGui::DockBuilderSplitNode(live_sensor_top, ImGuiDir_Right, 0.5, nullptr, &live_sensor_tl);
  ImGuiID live_sensor_ml = live_sensor_middle;
  ImGuiID live_sensor_mr = ImGui::DockBuilderSplitNode(live_sensor_middle, ImGuiDir_Right, 0.5, nullptr, &live_sensor_ml);
  ImGuiID live_sensor_bl = live_sensor_bottom;
  ImGuiID live_sensor_br = ImGui::DockBuilderSplitNode(live_sensor_bottom, ImGuiDir_Right, 0.5, nullptr, &live_sensor_bl);

  ImGuiID top_right_panel = right_panel;
  ImGuiID bottom_right_panel = ImGui::DockBuilderSplitNode(right_panel, ImGuiDir_Down, 0.5, nullptr, &top_right_panel);

  ImGuiID left_top_right_panel = top_right_panel;
  ImGuiID right_top_right_panel = ImGui::DockBuilderSplitNode(top_right_panel, ImGuiDir_Right, 0.5, nullptr, &left_top_right_panel);

  ImGuiID left_bottom_right_panel = bottom_right_panel;
  ImGuiID right_bottom_right_panel = ImGui::DockBuilderSplitNode(bottom_right_panel, ImGuiDir_Right, 0.5, nullptr, &left_bottom_right_panel);

  ImGui::DockBuilderDockWindow(IMU_ACCEL_PANEL, live_sensor_tl);
  ImGui::DockBuilderDockWindow(IMU_GYRO_PANEL, live_sensor_tr);
  ImGui::DockBuilderDockWindow(MAG_PANEL, live_sensor_ml);
  ImGui::DockBuilderDockWindow(GPS_VERT_PANEL, live_sensor_mr); // ACCEL_BIAS_PANEL
  ImGui::DockBuilderDockWindow(GPS_POS_PANEL, live_sensor_bl);
  ImGui::DockBuilderDockWindow(GPS_VEL_PANEL, live_sensor_br);

  ImGui::DockBuilderDockWindow(EST_POS_PANEL, left_top_right_panel);
  ImGui::DockBuilderDockWindow(EST_ROT_PANEL, right_top_right_panel);
  ImGui::DockBuilderDockWindow(CONTROLLER_OUTPUT_PANEL, left_bottom_right_panel);
  ImGui::DockBuilderDockWindow(GIMBAL_OUTPUT_PANEL, right_bottom_right_panel);

  ImGui::DockBuilderFinish(dockspace_id);
}

void gnc_panel() {
  ImGui::Begin(GNC_PANEL);

  ImGuiID dockspace_id = ImGui::GetID("GNC_Dockspace");
  ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_AutoHideTabBar);

  static bool first_time = true;
  if (first_time || (ImGui::IsKeyPressed(ImGuiKey_R))) {
    build_gnc_dock_layout(dockspace_id);
    first_time = false;
  }

  // create each window
  imu_accel_panel();
  imu_gyro_panel();
  mag_panel();
  // gyro_bias_panel();
  // accel_bias_panel();
  // mag_bias_panel();
  gps_pos_panel();
  gps_vel_panel();
  gps_vert_panel();
  estimated_pos_panel();
  estimated_orientation_panel();
  controller_output_panel();
  gimbal_output_panel();

  ImGui::End();
}