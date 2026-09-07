#pragma once

#include <Arduino.h>

#include "SPI.h"

// UARTS
// Primary serial output (UART5)
#define PIN_HW_COMM_SERIAL_RX PB12
#define PIN_HW_COMM_SERIAL_TX PB13

// Fallback serial output (UART3)
#define PIN_HW_FALLBACK_SERIAL_RX PB11
#define PIN_HW_FALLBACK_SERIAL_TX PB10

// RS485 Busses (UART6 and UART2)
// these are declared in ec_main
extern HardwareSerial RS485_6; // RS485 on UART 6
extern HardwareSerial RS485_2; // RS485 on UART 2

#define PIN_RS485_6_RX PG9
#define PIN_RS485_6_TX PG14
#define PIN_RS485_6_DE PG12
#define PIN_RS485_2_RX PD6
#define PIN_RS485_2_TX PD5
#define PIN_RS485_2_DE PD4

#define TVC_PITCH_RS485_BUS RS485_6
#define PIN_TVC_PITCH_SEL PF8
#define DRV_OX_RS485_BUS RS485_6 // unused
#define PIN_DRV_OX_SEL PF9       // unused
#define ENC_OX_RS485_BUS RS485_6
#define PIN_ENC_OX_SEL PF10

#define TVC_YAW_RS485_BUS RS485_2
#define PIN_TVC_YAW_SEL PD8
#define DRV_FU_RS485_BUS RS485_2 // unused
#define PIN_DRV_FU_SEL PD9       // unused
#define ENC_FU_RS485_BUS RS485_2
#define PIN_ENC_FU_SEL PD10

// SPI
#define PIN_PT_TC_SPI_1_MOSI PD7
#define PIN_PT_TC_SPI_1_MISO PB4
#define PIN_PT_TC_SPI_1_SCK PG11
#define PIN_PT_TC_SPI_3_MOSI PC12
#define PIN_PT_TC_SPI_3_MISO PC11
#define PIN_PT_TC_SPI_3_SCK PC10

// QSPI
#define PIN_QSPI_IO0 PD11
#define PIN_QSPI_IO1 PD12
#define PIN_QSPI_IO2 PF7
#define PIN_QSPI_IO3 PD13
#define PIN_QSPI_CLK PB2
#define PIN_QSPI_CS PG6

// FDCAN
#define PIN_CAN_TVC_RX PB5
#define PIN_CAN_TVC_TX PB6
#define PIN_CAN_FC_RX PH14
#define PIN_CAN_FC_TX PH13

// PWM (spark)
#define PIN_SPARK_PWM PC6
#define PIN_SPARK_TRIG PC13

// Zucrow Board
#define PIN_ZUCROW_BOARD_CS PE3 // TODO - replace with PT board definition

#define PIN_ZUCROW_BOARD_DO1 PF4
#define PIN_ZUCROW_BOARD_DO2 PF3

#define PIN_ZUCROW_BOARD_DI1 PI8
#define PIN_ZUCROW_BOARD_DI2 PF2
#define PIN_ZUCROW_BOARD_DI3 PE5
#define PIN_ZUCROW_BOARD_DI4 PE6

// Valve DOs
#define PIN_SV_BV_LATCH_ENABLE NC // TODO - flywire and assign

#define NUM_SV_BV_VALVES 16
#define PIN_SV_DO_1 PA4
#define PIN_SV_DO_2 PA5
#define PIN_SV_DO_3 PA6
#define PIN_SV_DO_4 PB0
#define PIN_SV_DO_5 PF11
#define PIN_SV_DO_6 PE12
#define PIN_SV_DO_7 PE13
#define PIN_SV_DO_8 PE14
#define PIN_SV_DO_9 PE15
#define PIN_BV_DO_10 PH6
#define PIN_BV_DO_11 PH7
#define PIN_BV_DO_12 PH8
#define PIN_BV_DO_13 PH9
#define PIN_BV_DO_14 PH10
#define PIN_BV_DO_15 PH11
#define PIN_BV_DO_16 PH12

#define PIN_VALVE_OE_INPUT PG1

// BOARDS
// these are declared in ec_main
extern SPIClass PT_TC_SPI_1;
extern SPIClass PT_TC_SPI_3;

#define NUM_PT_BOARDS 6

#define PT_BOARD_1_2_SPI_BUS PT_TC_SPI_3
#define PIN_PT_BOARD_1_2_CS PH15

#define PT_BOARD_3_4_SPI_BUS PT_TC_SPI_3
#define PIN_PT_BOARD_3_4_CS PG8

#define PT_BOARD_5_6_SPI_BUS PT_TC_SPI_3
#define PIN_PT_BOARD_5_6_CS PG7

#define PT_BOARD_7_8_SPI_BUS PT_TC_SPI_3
#define PIN_PT_BOARD_7_8_CS PG5

#define PT_BOARD_9_10_SPI_BUS PT_TC_SPI_1
#define PIN_PT_BOARD_9_10_CS PD3

#define PT_BOARD_11_12_SPI_BUS PT_TC_SPI_1
#define PIN_PT_BOARD_11_12_CS PG13

#define NUM_TC_CHIPS 6

#define TC_CHIP_1_SPI_BUS PT_TC_SPI_3
#define PIN_TC_CHIP_1_CS PD2

#define TC_CHIP_2_SPI_BUS PT_TC_SPI_3
#define PIN_TC_CHIP_2_CS PD1

#define TC_CHIP_3_SPI_BUS PT_TC_SPI_1
#define PIN_TC_CHIP_3_CS PI4

#define TC_CHIP_4_SPI_BUS PT_TC_SPI_1
#define PIN_TC_CHIP_4_CS PE1

#define TC_CHIP_5_SPI_BUS PT_TC_SPI_1
#define PIN_TC_CHIP_5_CS PI6

#define TC_CHIP_6_SPI_BUS PT_TC_SPI_1
#define PIN_TC_CHIP_6_CS PI7

// Unused
#define PIN_FAULT_TC_1 PD0
#define PIN_FAULT_TC_2 PI5
#define PIN_FAULT_TC_3 PE2
#define PIN_PT_SYNC PG10
#define PIN_ZUCROW_BOARD_EXTRA PE4
#define PIN_EXPOSED_GPIO_PF5 PF5
#define PIN_EXPOSED_GPIO_PF6 PF6
#define PIN_EXPOSED_GPIO_PF14 PF14
#define PIN_EXPOSED_GPIO_PF15 PF15

// utility macros
#define CONCAT_(a, b) a##b
#define CONCAT(a, b) CONCAT_(a, b)

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)
