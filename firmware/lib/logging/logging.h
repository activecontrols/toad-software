/**
 * @file logging.h
 * @brief Packed struct logging to NAND flash for GD5F1GQ5UEYIGR chip
 *
 * @author Daniel Proano (dproano@purdue.edu)
 */

#pragma once

#include "bad_page_table.h"
#include "flash.h"

#include <stddef.h>
#include <stdint.h>

#define LOG_EMPTY_BYTE 0xFFu

namespace Logging {

bool begin();

bool erase();

bool write(uint8_t struct_id, uint8_t *data, size_t len);

bool flush();

flash_error_t read_page(uint32_t page_addr, uint8_t *out);

bool dump();

} // namespace Logging
