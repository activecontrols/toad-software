#pragma once

#include <stddef.h>
#include <stdint.h>

enum flash_error_t { FLASH_ERROR_SUCCESS, FLASH_ERROR_FAIL, FLASH_ERROR_TIMED_OUT };

// Usable (ECC-covered) bytes per page. ECC is left enabled (chip default), so this
// does not include the OOB/spare area - the chip's on-die ECC engine handles that
// transparently on program/read.
#define FLASH_PAGE_SIZE (2048U)

struct __attribute__((packed)) flash_status_a_t {
  uint8_t RESERVED1 : 2;
  uint8_t ECCS : 2;
  uint8_t P_FAIL : 1;
  uint8_t E_FAIL : 1;
  uint8_t WEL : 1;
  uint8_t OIP : 1;
};

struct __attribute__((packed)) flash_status_b_t {
  uint8_t RESERVED1 : 2;
  uint8_t ECCS : 2;
  uint8_t BPS : 1;
  uint8_t RESERVED2 : 3;
};

namespace Flash {

// Initializes the QSPI peripheral and configures the NAND (reset, read ID, enable quad mode).
void begin();

// Reads one full page. Not flight-loop timing critical - internally does a
// Page-Read-To-Cache followed by a Read-From-Cache.
// addr is a page index (row address), not a byte offset.
flash_error_t read_page(uint32_t addr, uint8_t *out);

// Loads data into the NAND's internal program cache via PROGRAM LOAD.
// col_addr is the byte offset *within the cache* (0..FLASH_PAGE_SIZE-1), NOT a
// memory-array address.
flash_error_t write_to_cache(uint32_t col_addr, uint8_t *data, size_t len);

// Commits the current cache contents to the array at page addr (WRITE ENABLE + PROGRAM EXECUTE),
// after polling GET FEATURES until the chip is ready for a new write transaction.
flash_error_t program(uint32_t addr, uint32_t timeout_us = 1000);

// Erases the whole block containing addr. Required before addr's block can be programmed again.
flash_error_t erase_block(uint32_t addr, uint32_t timeout_us = 10000);

// Reads both status registers. status_a alone is expected to be sufficient to tell whether
// the last read/program failed (see ECCS/E_FAIL/P_FAIL); status_b may be left null.
flash_error_t read_status(flash_status_a_t *status_a, flash_status_b_t *status_b);

}; // namespace Flash
