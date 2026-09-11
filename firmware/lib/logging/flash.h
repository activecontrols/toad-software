#pragma once

#include <stddef.h>
#include <stdint.h>

#define FLASH_PAGE_SIZE (2048U)

typedef enum { 
  FLASH_ERROR_SUCCESS, 
  FLASH_ERROR_FAIL, 
  FLASH_ERROR_TIMED_OUT 
} flash_error_t;

typedef struct __attribute__((packed)) {
  uint8_t OIP : 1;
  uint8_t WEL : 1;
  uint8_t E_FAIL : 1;
  uint8_t P_FAIL : 1;
  uint8_t ECCS : 2;
  uint8_t RESERVED1 : 2;
} flash_status_a_t;

typedef struct __attribute__((packed)) {
  uint8_t RESERVED1 : 3;
  uint8_t BPS : 1;
  uint8_t ECCS : 2;
  uint8_t RESERVED2 : 2;
} flash_status_b_t;

namespace Flash {

void begin();

flash_error_t read_page(uint32_t addr, uint8_t *out);

flash_error_t write_to_cache(uint32_t col_addr, uint8_t *data, size_t len);

flash_error_t program(uint32_t addr, uint32_t timeout_us = 1000);

flash_error_t erase_block(uint32_t addr, uint32_t timeout_us = 10000);

flash_error_t read_status(flash_status_a_t *status_a, flash_status_b_t *status_b);

}; // namespace Flash
