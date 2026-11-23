#pragma once

#include <cstdint>

/*
Bootloader API
*/
namespace Boot_API
{
bool bank1_erase(void);
bool sync_frame(void);

// RM[22] - addr must be 16-bit aligned
// len does not need to be a multiple of 16 bytes - will be automatically padded
// len must be no greater than 256 bytes
bool write_memory(uint32_t addr, uint8_t* data, uint16_t len);
}