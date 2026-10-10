/**
 * @file logging.h
 * @brief NAND flash header for GD5F1GQ5UEYIGR chip
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#pragma once

#include "flash.h"
#include "bad_page_table.h"

namespace Logging {

bool begin();

bool write(uint8_t struct_id, uint8_t *data, size_t len);

bool 

}