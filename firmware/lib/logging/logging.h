#pragma once

#include "flash.h"
#include <stdint.h>

namespace Logging {

// Calls BPT::begin() and registers the log_arm/dump_flash/factory_scan serial commands.
// Does NOT scan for factory-bad blocks or erase anything by itself - that only happens
// once log_arm is confirmed (see cmd_log_arm in logging.cpp).
void begin();

bool is_armed();

// Writes struct_id, then the len bytes of data, as one atomic entry that never crosses a
// page boundary - so later, a reader can always tell which struct type follows by reading
// one id byte at a time. No-op if logging isn't armed. Automatically advances past bad
// pages/blocks (per BPT) and marks a page bad if writing to it fails.
void write(uint8_t struct_id, uint8_t *data, uint32_t len);

// Flushes any partially-filled page still sitting in the NAND's program cache, and disarms.
void complete();

// addr is a page index. Skips (fails immediately on) pages already marked bad; marks a page
// bad if reading it comes back with an uncorrectable ECC error.
// NOTE: signature matches Jacob's spec verbatim (uint32_t* data) - data is expected to point
// to a buffer at least FLASH_PAGE_SIZE bytes long; worth double-checking this shouldn't be
// uint8_t* instead.
flash_error_t read_page(uint32_t addr, uint32_t *data);

} // namespace Logging
