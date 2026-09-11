#pragma once

#include <stdint.h>

namespace BPT {

// Loads the Bad Page Table from internal flash (Bank 2, Sector 0 - see ldscript.ld).
// If the table has not been initialized yet, prompts the user before creating a new table.
// Must be called before any other BPT functions.
void begin();

// Returns whether the block containing page_addr is currently marked good.
// Bad NAND is tracked at block granularity even though the interface takes a page address.
bool is_page_good(uint32_t page_addr);

// Marks the entire block containing page_addr as bad and saves the updated table to
// STM32 internal flash.
void mark_page_bad(uint32_t page_addr);

// Returns whether the initial factory bad-block scan has completed successfully.
// Logging should not be allowed to erase or program NAND until this returns true.
bool factory_scan_complete();

// Marks the initial factory scan as complete in the persistent BPT header.
// This should only be called after every NAND block has been successfully checked.
bool mark_factory_scan_complete();

} // namespace BPT