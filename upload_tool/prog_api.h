#pragma once

#include <optional>
#include <cstdint>

namespace Prog_API
{
bool enter_bootloader(void);
bool enter_app(void);
bool spi_transfer(const uint8_t* mosi, uint8_t* miso, size_t len);

bool receive_frame(void); // receive a frame into the internal receive buffer
}