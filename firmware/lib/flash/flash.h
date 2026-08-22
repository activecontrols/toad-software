#pragma once

#include <stdint.h>

#define SECTOR(addr) ((addr >> 12) & 0xFFF)
#define PAGE(addr) ((addr >> 8) & 0xFFFF)
#define PAGE_SIZE 256

#define FLASH_MAX_ADDR 0xFFFFFF

namespace Flash {
void begin();

bool wait_for_wip(unsigned long max_delay = 50);
bool sector_erase(uint32_t addr);
bool chip_erase();
bool page_program(uint32_t addr, uint8_t *data, uint32_t len);

bool read(uint32_t addr, uint32_t len, uint8_t *out);

}; // namespace Flash