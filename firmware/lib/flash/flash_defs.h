#pragma once

#define CMD_EQIO 0x35
#define CMD_RDID 0x9F
#define CMD_QPIID 0xAF
#define CMD_RES 0xAB
#define CMD_WREN 0x06
#define CMD_WRDI 0x04
#define CMD_RDSR 0x05
#define CMD_RDSCUR 0x0c

#define CMD_PP 0x02

#define CMD_4READ 0xEB

// sector erase
#define CMD_SE 0x20

// block erase 32KB
#define CMD_BE32 0x52

// block erase 64KB
#define CMD_BE64 0xD8

// chip erase
#define CMD_CE 0x60

struct __packed flash_status_reg {
  uint8_t WIP : 1;  // write in progress bit
  uint8_t WEL : 1;  // write enable latch
  uint8_t BP0 : 1;  // BP0 level of protected block
  uint8_t BP1 : 1;  // BP1 level of protected block
  uint8_t BP2 : 1;  // BP2 level of protected block
  uint8_t BP3 : 1;  // BP3 level of protected block
  uint8_t QE : 1;   // quadspi enable
  uint8_t SRWD : 1; // status register write protect
};

struct __packed flash_config_reg {
  uint8_t ODS : 3; // output driver strength - see table 9 from datasheet
  uint8_t TB : 1;  // top/bottom OTP bit
  uint8_t RESERVED : 2;
  uint8_t DC : 2; // configurable dummy cycle count for certain commands
};

struct __packed flash_security_reg {
  uint8_t factory_otp_indicator : 1;
  uint8_t user_otp_indicator : 1;
  uint8_t PSB : 1;
  uint8_t ESB : 1;
  uint8_t RESERVED : 1;
  uint8_t P_FAIL : 1;
  uint8_t E_FAIL : 1;
  uint8_t WPSEL : 1;
};