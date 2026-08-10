#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "pid_diagram.h"

// P&ID Graphics
void DrawValve(ImVec2 center, char orientation, const char *label, ImVec2 label_center, bool open, bool hovered);
void DrawBallValve(ImVec2 center, char orientation, const char *label, ImVec2 label_center, bool open, bool hovered);
void DrawThrottleValve(ImVec2 center, char orientation, const char *label, ImVec2 label_center, bool open, bool hovered);
void DrawReg(ImVec2 center, const char *label);
void DrawCheckValve(ImVec2 center, char orientation);
void DrawManualValve(ImVec2 center, const char *label);
void DrawTank(ImVec2 center, const char *label, ImColor col, float fillLevel);
void DrawLargeNozzle(ImVec2 center);
void DrawSmallNozzle(ImVec2 center, char orientation);
void DrawIgniter(ImVec2 center);
void DrawReadout(ImVec2 center, ImVec2 attach_point, const char *name, const char *unit, float reading, char attach_direction);

bool MouseInValveHitbox(ImVec2 center, char orientation);