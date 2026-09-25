#include "ui.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "ui_components.h"
#include "ui_graphics.h"
#include "ui_graphs.h"

void fsw_avi_panel() {
  ImGui::Begin(FSW_AVI_PANEL);

  ImGui::End();
}

void build_dock_layout(ImGuiID dockspace_id) {
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

  ImGui::DockBuilderDockWindow(FSW_AVI_PANEL, dockspace_id);
  ImGui::DockBuilderDockWindow(FLUIDS_PANEL, dockspace_id);
  ImGui::DockBuilderDockWindow(GNC_PANEL, dockspace_id);

  ImGui::DockBuilderFinish(dockspace_id);
}

void render_loop() {
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

  const ImGuiViewport *vp = ImGui::GetMainViewport();

  ImGui::SetNextWindowPos(vp->Pos);
  ImGui::SetNextWindowSize(vp->Size);
  ImGui::SetNextWindowViewport(vp->ID);

  ImGui::Begin("MainWindow", nullptr, flags);

  ImGuiID dockspace_id = ImGui::GetID("MyDockspace");
  ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_AutoHideTabBar);

  static bool first_time = true;
  if (first_time || (ImGui::IsKeyPressed(ImGuiKey_R))) {
    build_dock_layout(dockspace_id);
    first_time = false;
  }

  fsw_avi_panel();
  fluids_panel();
  gnc_panel();

  ImGui::End();
}

// TODO - lockout during autoseq
// TODO - pressure history viewer (with redlines)
// TODO - autoseq viewer
// TODO - autoseq stuff
