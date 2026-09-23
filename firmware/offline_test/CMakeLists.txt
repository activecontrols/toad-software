cmake_minimum_required(VERSION 3.20)

# Default to 32-bit target (x86-linux) to match embedded hardware architecture (e.g. ARM 32-bit Cortex-M)
if(NOT DEFINED VCPKG_TARGET_TRIPLET)
    set(VCPKG_TARGET_TRIPLET "x86-linux" CACHE STRING "")
endif()

# Auto-detect VCPKG toolchain if not explicitly passed
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

# Force 32-bit compilation on host to mirror 32-bit microcontroller memory model
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT DEFINED CMAKE_SYSTEM_NAME)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -m32")
endif()

project(toad_offline_test CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-m32)
add_link_options(-m32)

find_package(Boost REQUIRED COMPONENTS fiber context)

# Official ArduinoCore-API dependency
include(FetchContent)
FetchContent_Declare(
    arduino_core_api
    GIT_REPOSITORY https://github.com/arduino/ArduinoCore-API.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(arduino_core_api)

# Host build flags for ArduinoCore-API (skips weak embedded atexit, enables 32-bit pin mode)
add_compile_definitions(HOST EXTENDED_PIN_MODE)

# Include paths:
# 1. hal_mock/ takes top precedence
# 2. core/, bus/, peripherals/
# 3. Official ArduinoCore-API
# 4. Firmware lib/ and src/
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hal_mock
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/bus
    ${CMAKE_CURRENT_SOURCE_DIR}/peripherals
    ${CMAKE_CURRENT_SOURCE_DIR}/runner
    ${arduino_core_api_SOURCE_DIR}
    ${arduino_core_api_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)

# Core simulation static library
add_library(toad_sim_core STATIC
    core/VirtualClock.cpp
    core/FiberScheduler.cpp
    core/BusRegistry.cpp
    core/SimulatedGPIO.cpp
    hal_mock/Arduino.cpp
    hal_mock/HardwareSerial.cpp
    hal_mock/SPI.cpp
    bus/UartBus.cpp
    bus/UartSnooper.cpp
    bus/RS485Mux.cpp
    bus/SPIBus.cpp
    peripherals/AMT242AV_Sim.cpp
    peripherals/ADS131M02_Sim.cpp
    runner/UartTerminalHarness.cpp
    ${arduino_core_api_SOURCE_DIR}/api/Print.cpp
    ${arduino_core_api_SOURCE_DIR}/api/Stream.cpp
    ${arduino_core_api_SOURCE_DIR}/api/String.cpp
)

target_link_libraries(toad_sim_core PUBLIC
    Boost::fiber
    Boost::context
    pthread
)

enable_testing()

# Milestone 1: Virtual Clock & Fiber Scheduling test
add_executable(test_virtual_clock
    tests/test_virtual_clock.cpp
)
target_link_libraries(test_virtual_clock PRIVATE toad_sim_core)
add_test(NAME test_virtual_clock COMMAND test_virtual_clock)

# Milestone 2: Uart, UartBus & Snooper test
add_executable(test_uart_bus
    tests/test_uart_bus.cpp
)
target_link_libraries(test_uart_bus PRIVATE toad_sim_core)
add_test(NAME test_uart_bus COMMAND test_uart_bus)

# Milestone: RS485 Muxing & Pluggable Peripherals test
add_executable(test_rs485_mux
    tests/test_rs485_mux.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/throttle_valves/AMT242AV.cpp
)
target_link_libraries(test_rs485_mux PRIVATE toad_sim_core)
add_test(NAME test_rs485_mux COMMAND test_rs485_mux)

# Milestone: SPI Emulation & Pluggable Devices test
add_executable(test_spi_bus
    tests/test_spi_bus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/pressure_sensors/ADS131M02.cpp
)
target_link_libraries(test_spi_bus PRIVATE toad_sim_core)
add_test(NAME test_spi_bus COMMAND test_spi_bus)

# Standalone Terminal Interactive Harness
add_executable(uart_terminal_harness
    runner/uart_terminal_harness.cpp
)
target_link_libraries(uart_terminal_harness PRIVATE toad_sim_core)
