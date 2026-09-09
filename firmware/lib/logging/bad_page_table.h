#pragma once

#include <stdint.h>

namespace BPT {

// Loads the Bad Page Table from internal flash (Bank 2, Sector 0 - see ldscript.ld).
// Checks a magic number at the table's base; if missing, prompts over serial before
// initializing a fresh (all-good) table. Must be called before is_page_good()/mark_page_bad().
void begin();

// NAND bad-block marking is inherently block-granularity (the factory marker and any
// runtime ECC failure both apply to the whole block containing a page), so addr is
// mapped to its containing block internally even though it's expressed as a page index.
bool is_page_good(uint32_t addr);

void mark_page_bad(uint32_t addr);

} // namespace BPT
