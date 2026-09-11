#include "ui_components.h"
#include "imgui_internal.h"

void centered_text(const char *text) {
  ImGuiStyle &style = ImGui::GetStyle();

  float size = ImGui::CalcTextSize(text).x + style.FramePadding.x * 2.0f;
  float avail = ImGui::GetContentRegionAvail().x;

  float off = (avail - size) * 0.5;
  if (off > 0.0f)
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + off);

  ImGui::PushFont(NULL, 24);
  ImGui::Text(text);
  ImGui::PopFont();
}

ImU32 AdjustBrightness(ImU32 color, float factor) {
  ImVec4 c = ImGui::ColorConvertU32ToFloat4(color);
  c.x = ImClamp(c.x * factor, 0.0f, 1.0f);
  c.y = ImClamp(c.y * factor, 0.0f, 1.0f);
  c.z = ImClamp(c.z * factor, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(c);
}

bool rounded_button(const char *label, const ImVec2 &size, ImU32 color, float rounding) {
  ImGui::PushStyleColor(ImGuiCol_Button, color);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, AdjustBrightness(color, 1.2));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, AdjustBrightness(color, 0.7));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, rounding);

  bool clicked = ImGui::Button(label, size);

  ImGui::PopStyleVar();
  ImGui::PopStyleColor(3);

  return clicked;
}

void status_flag(char *text, bool state_ok, char *imgui_id) {
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.0f);

  ImVec4 red = ImVec4(204.0 / 255.0, 0.0f, 0.0f, 1.0f);
  ImVec4 green = ImVec4(0.0f, 153.0 / 255.0, 0.0f, 1.0f);

  if (state_ok) {
    ImGui::PushStyleColor(ImGuiCol_FrameBg, green);
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, green);
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, green);
  } else {
    ImGui::PushStyleColor(ImGuiCol_FrameBg, red);
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, red);
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, red);
  }

  ImGui::SetNextItemWidth(200.0f); // pixels
  ImGui::BeginDisabled();          // prevent editing
  ImGui::InputText(imgui_id, text, ImGuiInputTextFlags_ReadOnly);
  ImGui::EndDisabled();
  ImGui::PopStyleColor(3);
  ImGui::PopStyleVar();
}
