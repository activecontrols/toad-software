#pragma once

#include <Arduino.h>

#include "SPI.h"

// TODO - audit these once EC design is finalized

// UARTS
// Primary serial output (UART1)
#define PIN_HW_COMM_SERIAL_RX PA10
#define PIN_HW_COMM_SERIAL_TX PA9

// Fallback serial input from H7 (UART3)
#define PIN_HW_FALLBACK_SERIAL_RX PB11
#define PIN_HW_FALLBACK_SERIAL_TX PB10

// SPI
#define PIN_PROG_SPI_MOSI PB5
#define PIN_PROG_SPI_MISO PB4
#define PIN_PROG_SPI_SCK PB3
#define PIN_PROG_SPI_CS PC13

// ID Pin
#define PIN_PROG_ID PB1
#define PROG_ID_FLIGHT_CONTROLLER HIGH
#define PROG_ID_ENGINE_CONTROLLER LOW
enum prog_id_t { PROG_FLIGHT_CONTROLLER, PROG_ENGINE_CONTROLLER };

// BOOT / RESET Control
#define PIN_H7_BOOT PC0
#define PIN_H7_NRST PC1

// TODO CAN, TEST_LED