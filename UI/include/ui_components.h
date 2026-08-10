#pragma once
#include "imgui.h"

void centered_text(const char *text);
bool rounded_button(const char *label, const ImVec2 &size, ImU32 color, float rounding = 10.0f);
void colored_flag(char *text, bool state, ImVec4 on_color, ImVec4 off_color, char *imgui_id);
ImU32 AdjustBrightness(ImU32 color, float factor);