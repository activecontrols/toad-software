/**
 * @file bad_page_table.h
 * @brief Bad Page Table Header
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#pragma once

#include "flash_defs.h"
#include <stdint.h>

#define NUM_BITS (NAND_NUM_BLOCKS)
#define NUM_BYTES_TRACKING_BLOCKS (NUM_BITS / 8)
#define PAGE_MARKED_GOOD (0xFF)

typedef enum {
  PAGE_GOOD,
  PAGE_READ_ONLY,
  PAGE_BAD
} page_status_t;

namespace Bad_Page_Table {

bool begin();

page_status_t get_page_status(uint32_t address);

bool mark_page_bad(uint32_t address);

bool retire_block(uint32_t address);

}
