CXX      := g++
CXXFLAGS := -IUI/imgui -IUI/imgui/backends -IUI/implot -IUI/implot3d -IUI/include -MMD -MP

# the majority of this is just windows libraries
# the -static is because we were pulling in the wrong C++ .dlls (from the RUST ESP32 project), resulting in crazy crashes
LDFLAGS  := -luser32 -lgdi32 -ld3d11 -ldxgi -ld3dcompiler -ldwmapi -lole32 -luuid -lshlwapi -lsetupapi -lws2_32 -static

# Sources
SRC := \
	$(wildcard UI/*.cpp) \
	$(filter-out %demo.cpp,$(wildcard UI/imgui/*.cpp UI/implot/*.cpp UI/implot3d/*.cpp)) \
	UI/imgui/backends/imgui_impl_win32.cpp \
	UI/imgui/backends/imgui_impl_dx11.cpp

# Build paths
BUILD_DIR := build
OBJ := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRC))

TARGET := UI/app.exe

.PHONY: all clean

all: $(TARGET)

# Link
$(TARGET): $(OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

# Compile
$(OBJ): $(BUILD_DIR)/%.o: %.cpp
	@if not exist "$(dir $@)" mkdir "$(dir $@)"
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)
	@if exist $(TARGET) del /q $(TARGET)

# Include dependency files
-include $(OBJ:.o=.d)