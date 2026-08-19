#pragma once

#include <Arduino.h>

#include "SPI.h"

// TODO - define these once EC design is finalized

// UARTS
// Primary serial output (UART5)
#define PIN_HW_COMM_SERIAL_RX NC
#define PIN_HW_COMM_SERIAL_TX NC

// Fallback serial output (UART3)
#define PIN_HW_FALLBACK_SERIAL_RX NC
#define PIN_HW_FALLBACK_SERIAL_TX NC

// RS485 Busses (UART1 and UART4)
// these are declared in ec_main
extern HardwareSerial RS485_1; // RS485 on UART 1
extern HardwareSerial RS485_4; // RS485 on UART 4

// TODO - consider making an RS485 device struct that bundles these per device

#define PIN_RS485_1_RX NC
#define PIN_RS485_1_TX NC
#define PIN_RS485_4_RX NC
#define PIN_RS485_4_TX NC

#define OX_ENC_RS485_BUS RS485_1
#define PIN_OX_ENC_DE NC
#define PIN_OX_ENC_RE NC

#define FU_ENC_RS485_BUS RS485_4
#define PIN_FU_ENC_DE NC
#define PIN_FU_ENC_RE NC

// SPI
#define PIN_PT_TC_SPI_1_MOSI NC
#define PIN_PT_TC_SPI_1_MISO NC
#define PIN_PT_TC_SPI_1_SCK NC
#define PIN_PT_TC_SPI_3_MOSI NC
#define PIN_PT_TC_SPI_3_MISO NC
#define PIN_PT_TC_SPI_3_SCK NC

// DOs
#define PIN_SV_BV_LATCH_ENABLE NC

#define NUM_SV_BV_VALVES 16
#define PIN_SV_DO_1 NC
#define PIN_SV_DO_2 NC
#define PIN_SV_DO_3 NC
#define PIN_SV_DO_4 NC
#define PIN_SV_DO_5 NC
#define PIN_SV_DO_6 NC
#define PIN_SV_DO_7 NC
#define PIN_SV_DO_8 NC
#define PIN_SV_DO_9 NC
#define PIN_BV_DO_10 NC
#define PIN_BV_DO_11 NC
#define PIN_BV_DO_12 NC
#define PIN_BV_DO_13 NC
#define PIN_BV_DO_14 NC
#define PIN_BV_DO_15 NC
#define PIN_BV_DO_16 NC

// BOARDS
// these are declared in ec_main
extern SPIClass PT_TC_SPI_1;
extern SPIClass PT_TC_SPI_3;

#define NUM_PT_BOARDS 6

#define PT_BOARD_1_2_SPI_BUS PT_TC_SPI_1
#define PIN_PT_BOARD_1_2_CS NC

#define PT_BOARD_3_4_SPI_BUS PT_TC_SPI_1
#define PIN_PT_BOARD_3_4_CS NC

#define PT_BOARD_5_6_SPI_BUS PT_TC_SPI_1
#define PIN_PT_BOARD_5_6_CS NC

#define PT_BOARD_7_8_SPI_BUS PT_TC_SPI_3
#define PIN_PT_BOARD_7_8_CS NC

#define PT_BOARD_9_10_SPI_BUS PT_TC_SPI_3
#define PIN_PT_BOARD_9_10_CS NC

#define PT_BOARD_11_12_SPI_BUS PT_TC_SPI_3
#define PIN_PT_BOARD_11_12_CS NC

#define NUM_TC_CHIPS 4

#define TC_BOARD_1_SPI_BUS PT_TC_SPI_1
#define PIN_TC_BOARD_1_CS NC

#define TC_BOARD_2_SPI_BUS PT_TC_SPI_1
#define PIN_TC_BOARD_2_CS NC

#define TC_BOARD_3_SPI_BUS PT_TC_SPI_3
#define PIN_TC_BOARD_3_CS NC

#define TC_BOARD_4_SPI_BUS PT_TC_SPI_3
#define PIN_TC_BOARD_4_CS NC

// utility macros
#define CONCAT_(a, b) a##b
#define CONCAT(a, b) CONCAT_(a, b)

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)
