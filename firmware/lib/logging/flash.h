/**
 * @file flash.h
 * @brief NAND flash header for GD5F1GQ5UEYIGR chip
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

enum flash_error_t { 
  FLASH_SUCCESS, 
  FLASH_READY, 
  FLASH_FAIL, 
  FLASH_TIMED_OUT, 
  FLASH_BAD_BLOCK
};

struct __attribute__((packed)) flash_status_a_t {
  uint8_t OIP : 1;      // Operation in progress
  uint8_t WEL : 1;      // Write Enable Latch
  uint8_t E_FAIL : 1;   // Erase Fail (can also indicate a user tried to erase a protected region)
  uint8_t P_FAIL : 1;   // Program fail (can also indicate a user tried to program a protected region)
  uint8_t ECCS : 2;     // Error Correction Code Status
  uint8_t RESERVED1 : 2;
};

struct __attribute__((packed)) flash_status_b_t {
  uint8_t RESERVED1 : 3;
  uint8_t BPS : 1;      // Block Protection Status (when BPS is 1, selected block is protected)
  uint8_t ECCSE : 2;    // Error Correction Code Status
  uint8_t RESERVED2 : 2;
};

namespace Flash {

bool begin();

flash_error_t read(uint32_t row_addr, uint32_t col_addr, uint8_t *out, size_t len);

flash_error_t write_to_cache(uint32_t col_addr, uint8_t *data, size_t len);

flash_error_t program(uint32_t row_addr);

flash_error_t check_program_status();

flash_error_t erase_block(uint32_t row_addr);

}; // namespace Flash
