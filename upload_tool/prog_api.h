#pragma once

#include <optional>
#include <cstdint>

/*
Programmer MCU API
*/
namespace Prog_API
{
bool enter_bootloader(void);
bool enter_app(void);

// Note: miso may be nullptr, in which case it is ignored
bool spi_transfer(const uint8_t* mosi, uint8_t* miso, size_t len);
}