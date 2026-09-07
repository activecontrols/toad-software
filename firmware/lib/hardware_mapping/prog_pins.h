#pragma once

#include <Arduino.h>

#include "SPI.h"

// TODO - audit these once EC design is finalized

// UARTS
// Primary serial output (UART2)
#define PIN_HW_COMM_SERIAL_RX PA3
#define PIN_HW_COMM_SERIAL_TX PA2

// Fallback serial input from H7 (UART1)
#define PIN_HW_FALLBACK_SERIAL_RX PC5
#define PIN_HW_FALLBACK_SERIAL_TX PC4

// SPI
#define PIN_PROG_SPI_MOSI PC12
#define PIN_PROG_SPI_MISO PC11
#define PIN_PROG_SPI_SCK PC10
#define PIN_PROG_SPI_CS PA15

// FDCAN
#define PIN_CAN_GSE_RX PB12
#define PIN_CAN_GSE_TX PB13

// ID Pin
#define PIN_PROG_ID PB1
#define PROG_ID_FLIGHT_CONTROLLER HIGH
#define PROG_ID_ENGINE_CONTROLLER LOW
enum prog_id_t { PROG_FLIGHT_CONTROLLER, PROG_ENGINE_CONTROLLER };

// BOOT / RESET Control
#define PIN_H7_BOOT PC0
#define BOOT_MODE_RUN LOW    // default: don't enter bootloader on STM32H7
#define BOOT_MODE_FLASH HIGH // enter the STM32H7 bootloader for firmware flashing

#define PIN_H7_NRST PC1
#define NRST_MODE_RUN HIGH // default: don't reset the STM32H7
#define NRST_MODE_RST LOW  // reset the STM32H7

// Unused
#define PIN_EXPOSED_GPIO_B3 PB3
#define PIN_EXPOSED_GPIO_B4 PB4
