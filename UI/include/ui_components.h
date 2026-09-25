#pragma once
#include "imgui.h"

void centered_text(const char *text);
bool rounded_button(const char *label, const ImVec2 &size, ImU32 color, float rounding = 10.0f);
void status_flag(char *text, bool state_ok, char *imgui_id);
ImU32 AdjustBrightness(ImU32 color, float factor);