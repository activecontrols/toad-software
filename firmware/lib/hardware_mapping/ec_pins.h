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
#define PIN_RS485_1_RX NC
#define PIN_RS485_1_TX NC
#define PIN_RS485_4_RX NC
#define PIN_RS485_4_TX NC

// SPI
#define PIN_PT_TC_SPI_1_MOSI NC
#define PIN_PT_TC_SPI_1_MISO NC
#define PIN_PT_TC_SPI_1_SCK NC
#define PIN_PT_TC_SPI_3_MOSI NC
#define PIN_PT_TC_SPI_3_MISO NC
#define PIN_PT_TC_SPI_3_SCK NC

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
