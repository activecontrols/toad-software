/**
 * @file bad_page_table.h
 * @brief Bad Page Table Header
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#pragma once

#include "flash_defs.h"
#include <stdint.h>

enum page_status_t {
  PAGE_GOOD,      // page can be read and written
  PAGE_READ_ONLY, // page was written in a block that then went bad, now read-only to save data
  PAGE_BAD        // page is never read or written
};

namespace Bad_Page_Table {

// Checks the table is set up in STM32 flash, otherwise runs the one time
// factory bad block scan (requires user confirmation over serial)
bool begin();

page_status_t get_page_status(uint32_t row_addr);

// Records that a program failed at row_addr. Earlier pages in its block
// become read only, and it and every later page in the block become bad
bool mark_page_bad(uint32_t row_addr);

// Marks every page in the block containing row_addr bad,
// used once the block's read only data is no longer needed
bool retire_block(uint32_t row_addr);

} // namespace Bad_Page_Table
