#pragma once

#include "flash.h"

#include <stdint.h>

namespace Logging {

// Initializes the logging subsystem, loads the Bad Page Table, and registers the
// log_arm, dump_flash, and factory_scan serial commands.
//
// This does not scan, erase, or program the NAND by itself. NAND erase/program
// operations remain locked until the factory bad-block scan has completed.
void begin();

bool is_armed();

// Writes struct_id followed by len bytes of data as a single log entry. Entries are
// never split across NAND page boundaries, so each entry can be decoded later by
// reading the id byte before the structure data.
//
// Does nothing if logging is not armed. When the current page fills, it is programmed
// and logging advances to the next page whose containing block is not marked bad.
void write(uint8_t struct_id, uint8_t *data, uint32_t len);

// Programs any partially-filled page still in the NAND cache and disables logging.
void complete();

// Reads one NAND page into data. addr is a NAND page index.
//
// Pages whose containing block is already marked bad are rejected immediately.
// If the read reports an uncorrectable ECC failure, the containing block is added
// to the Bad Page Table.
//
// The uint32_t* parameter matches the interface in Jacob's original spec, although
// the buffer is used as raw page data and must be at least FLASH_PAGE_SIZE bytes.
flash_error_t read_page(uint32_t addr, uint32_t *data);

} // namespace Logging