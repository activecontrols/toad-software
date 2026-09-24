/**
 * @file flash_defs.hpp
 * @brief Bad Page Table Header
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#pragma once

#include <stdint.h>
#include "flash_defs.h"

#define NUM_BITS (NAND_NUM_BLOCKS)
#define NUM_BYTES_TRACKING_BLOCKS (NUM_BITS / 8)

namespace Bad_Page_Table {

bool begin();

bool scan_factory_bad_blocks(void);

bool is_page_good(uint32_t address);

void mark_page_bad(uint32_t address);

}

