#pragma once

#include <Arduino.h>

#include "SPI.h"

// UARTS
// Primary serial output (UART5)
constexpr pin_size_t PIN_HW_COMM_SERIAL_RX = PB12;
constexpr pin_size_t PIN_HW_COMM_SERIAL_TX = PB13;

// Fallback serial output (UART3)
constexpr pin_size_t PIN_HW_FALLBACK_SERIAL_RX = PB11;
constexpr pin_size_t PIN_HW_FALLBACK_SERIAL_TX = PB10;

// RS485 Busses (UART6 and UART2)
// these are declared in ec_main
extern Uart RS485_6; // RS485 on UART 6
extern Uart RS485_2; // RS485 on UART 2

constexpr pin_size_t PIN_RS485_6_RX = PG9;
constexpr pin_size_t PIN_RS485_6_TX = PG14;
constexpr pin_size_t PIN_RS485_6_DE = PG12;
constexpr pin_size_t PIN_RS485_2_RX = PD6;
constexpr pin_size_t PIN_RS485_2_TX = PD5;
constexpr pin_size_t PIN_RS485_2_DE = PD4;

inline Uart &TVC_PITCH_RS485_BUS = RS485_6;
constexpr pin_size_t PIN_TVC_PITCH_SEL = PF8;
inline Uart &DRV_OX_RS485_BUS = RS485_6;   // unused
constexpr pin_size_t PIN_DRV_OX_SEL = PF9; // unused
inline Uart &ENC_OX_RS485_BUS = RS485_6;
constexpr pin_size_t PIN_ENC_OX_SEL = PF10;

inline Uart &TVC_YAW_RS485_BUS = RS485_2;
constexpr pin_size_t PIN_TVC_YAW_SEL = PD8;
inline Uart &DRV_FU_RS485_BUS = RS485_2;   // unused
constexpr pin_size_t PIN_DRV_FU_SEL = PD9; // unused
inline Uart &ENC_FU_RS485_BUS = RS485_2;
constexpr pin_size_t PIN_ENC_FU_SEL = PD10;

// SPI
constexpr pin_size_t PIN_PT_TC_SPI_1_MOSI = PD7;
constexpr pin_size_t PIN_PT_TC_SPI_1_MISO = PB4;
constexpr pin_size_t PIN_PT_TC_SPI_1_SCK = PG11;
constexpr pin_size_t PIN_PT_TC_SPI_3_MOSI = PC12;
constexpr pin_size_t PIN_PT_TC_SPI_3_MISO = PC11;
constexpr pin_size_t PIN_PT_TC_SPI_3_SCK = PC10;

// QSPI
constexpr pin_size_t PIN_QSPI_IO0 = PD11;
constexpr pin_size_t PIN_QSPI_IO1 = PD12;
constexpr pin_size_t PIN_QSPI_IO2 = PF7;
constexpr pin_size_t PIN_QSPI_IO3 = PD13;
constexpr pin_size_t PIN_QSPI_CLK = PB2;
constexpr pin_size_t PIN_QSPI_CS = PG6;

// FDCAN
constexpr pin_size_t PIN_CAN_TVC_RX = PB5;
constexpr pin_size_t PIN_CAN_TVC_TX = PB6;
constexpr pin_size_t PIN_CAN_FC_RX = PH14;
constexpr pin_size_t PIN_CAN_FC_TX = PH13;

// PWM (spark)
constexpr pin_size_t PIN_SPARK_PWM = PC6;
constexpr pin_size_t PIN_SPARK_TRIG = PC13;

// Zucrow Board
constexpr pin_size_t PIN_ZUCROW_BOARD_CS = PE3; // TODO - replace with PT board definition

constexpr pin_size_t PIN_ZUCROW_BOARD_DO1 = PF4;
constexpr pin_size_t PIN_ZUCROW_BOARD_DO2 = PF3;

constexpr pin_size_t PIN_ZUCROW_BOARD_DI1 = PI8;
constexpr pin_size_t PIN_ZUCROW_BOARD_DI2 = PF2;
constexpr pin_size_t PIN_ZUCROW_BOARD_DI3 = PE5;
constexpr pin_size_t PIN_ZUCROW_BOARD_DI4 = PE6;

// Valve DOs
constexpr pin_size_t PIN_SV_BV_LATCH_ENABLE = NC; // TODO - flywire and assign

constexpr size_t NUM_SV_BV_VALVES = 16;
constexpr pin_size_t PIN_SV_DO_1 = PA4;
constexpr pin_size_t PIN_SV_DO_2 = PA5;
constexpr pin_size_t PIN_SV_DO_3 = PA6;
constexpr pin_size_t PIN_SV_DO_4 = PB0;
constexpr pin_size_t PIN_SV_DO_5 = PF11;
constexpr pin_size_t PIN_SV_DO_6 = PE12;
constexpr pin_size_t PIN_SV_DO_7 = PE13;
constexpr pin_size_t PIN_SV_DO_8 = PE14;
constexpr pin_size_t PIN_SV_DO_9 = PE15;
constexpr pin_size_t PIN_BV_DO_10 = PH6;
constexpr pin_size_t PIN_BV_DO_11 = PH7;
constexpr pin_size_t PIN_BV_DO_12 = PH8;
constexpr pin_size_t PIN_BV_DO_13 = PH9;
constexpr pin_size_t PIN_BV_DO_14 = PH10;
constexpr pin_size_t PIN_BV_DO_15 = PH11;
constexpr pin_size_t PIN_BV_DO_16 = PH12;

constexpr pin_size_t PIN_VALVE_OE_INPUT = PG1;

// BOARDS
// these are declared in ec_main
extern SPIClass PT_TC_SPI_1;
extern SPIClass PT_TC_SPI_3;

constexpr size_t NUM_PT_BOARDS = 6;

inline SPIClass &PT_BOARD_1_2_SPI_BUS = PT_TC_SPI_3;
constexpr pin_size_t PIN_PT_BOARD_1_2_CS = PH15;

inline SPIClass &PT_BOARD_3_4_SPI_BUS = PT_TC_SPI_3;
constexpr pin_size_t PIN_PT_BOARD_3_4_CS = PG8;

inline SPIClass &PT_BOARD_5_6_SPI_BUS = PT_TC_SPI_3;
constexpr pin_size_t PIN_PT_BOARD_5_6_CS = PG7;

inline SPIClass &PT_BOARD_7_8_SPI_BUS = PT_TC_SPI_3;
constexpr pin_size_t PIN_PT_BOARD_7_8_CS = PG5;

inline SPIClass &PT_BOARD_9_10_SPI_BUS = PT_TC_SPI_1;
constexpr pin_size_t PIN_PT_BOARD_9_10_CS = PD3;

inline SPIClass &PT_BOARD_11_12_SPI_BUS = PT_TC_SPI_1;
constexpr pin_size_t PIN_PT_BOARD_11_12_CS = PG13;

constexpr size_t NUM_TC_CHIPS = 6;

inline SPIClass &TC_CHIP_1_SPI_BUS = PT_TC_SPI_3;
constexpr pin_size_t PIN_TC_CHIP_1_CS = PD2;

inline SPIClass &TC_CHIP_2_SPI_BUS = PT_TC_SPI_3;
constexpr pin_size_t PIN_TC_CHIP_2_CS = PD1;

inline SPIClass &TC_CHIP_3_SPI_BUS = PT_TC_SPI_1;
constexpr pin_size_t PIN_TC_CHIP_3_CS = PI4;

inline SPIClass &TC_CHIP_4_SPI_BUS = PT_TC_SPI_1;
constexpr pin_size_t PIN_TC_CHIP_4_CS = PE1;

inline SPIClass &TC_CHIP_5_SPI_BUS = PT_TC_SPI_1;
constexpr pin_size_t PIN_TC_CHIP_5_CS = PI6;

inline SPIClass &TC_CHIP_6_SPI_BUS = PT_TC_SPI_1;
constexpr pin_size_t PIN_TC_CHIP_6_CS = PI7;

// Unused
constexpr pin_size_t PIN_FAULT_TC_1 = PD0;
constexpr pin_size_t PIN_FAULT_TC_2 = PI5;
constexpr pin_size_t PIN_FAULT_TC_3 = PE2;
constexpr pin_size_t PIN_PT_SYNC = PG10;
constexpr pin_size_t PIN_ZUCROW_BOARD_EXTRA = PE4;
constexpr pin_size_t PIN_EXPOSED_GPIO_PF5 = PF5;
constexpr pin_size_t PIN_EXPOSED_GPIO_PF6 = PF6;
constexpr pin_size_t PIN_EXPOSED_GPIO_PF14 = PF14;
constexpr pin_size_t PIN_EXPOSED_GPIO_PF15 = PF15;

// utility macros
#define CONCAT_(a, b) a##b
#define CONCAT(a, b) CONCAT_(a, b)

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)
