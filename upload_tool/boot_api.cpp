#include "boot_api.h"
#include "prog_api.h"
#include "util.h"
#include <cstring>

namespace Boot_API
{
// start-of-frame bytes
constexpr uint8_t MOSI_SOF = 0x5A;
constexpr uint8_t MISO_SOF = 0xA5;

constexpr uint8_t NULL_BYTE = 0x00;
constexpr uint8_t ACK = 0x79;
constexpr uint8_t NACK = 0x1F;

// From AN[6]
bool get_ack(void)
{
    uint8_t res = 0;
    ExitOnFail(Prog_API::spi_transfer(&NULL_BYTE, &res, sizeof(NULL_BYTE)));

    // from AN[6] - loop until we receive either ACK or NACK
    for (;;)
    {
        ExitOnFail(Prog_API::spi_transfer(&NULL_BYTE, &res, sizeof(NULL_BYTE)));

        if (res == ACK)
        {
            // reply with ACK of our own
            ExitOnFail(Prog_API::spi_transfer(&ACK, &res, sizeof(ACK)));
            return true;
        }

        if (res == NACK)
        {
            std::cerr << "Received NACK" << std::endl;

            // reply with ACK of our own (not a mistake - see AN[6])
            ExitOnFail(Prog_API::spi_transfer(&ACK, &res, sizeof(ACK)));
            return false;
        }
    }

    return false; // never reached
}

bool bank1_erase(void)
{
    static constexpr uint8_t command[3] = {MOSI_SOF, 0x44, 0xBB};

    ExitOnFail(Prog_API::spi_transfer(command, nullptr, sizeof(command)));

    ExitOnFail(get_ack());

    // AN[25] erase bank 1
    constexpr uint16_t n_pages = 0xFFFE;

    uint8_t mosi_data[3] = {
        (n_pages >> 8) & 0xFF,
        n_pages & 0xFF
    };

    mosi_data[2] = mosi_data[0] ^ mosi_data[1];

    ExitOnFail(Prog_API::spi_transfer(mosi_data, nullptr, sizeof(mosi_data)));

    // Wait for ACK or NACK
    ExitMsgOnFail(get_ack(), "bank1_erase(): bootloader rejected bank erase with NACK");

    return true;
}

bool sync_frame(void)
{
    uint8_t res = 0;
    ExitOnFail(Prog_API::spi_transfer(&MOSI_SOF, &res, sizeof(MOSI_SOF)));

    ExitOnFail(res == MISO_SOF);

    ExitOnFail(get_ack());

    return true;
}

bool write_memory(uint32_t addr, uint8_t* data, uint16_t len)
{
    ExitMsgOnFail(addr % 2 == 0, "write_memory(): addr must be a multiple of 2");
    ExitMsgOnFail(len != 0, "write_memory(): len must be nonzero");
    ExitMsgOnFail(len > 256, "write_memory(): len must be no greater than 256 bytes");

    uint16_t byte_count_to_write = len;

    if (len % 2 != 0)
    {
        // pad with 1 byte so the data length is a multiple of 2 (RM[22])
        byte_count_to_write = len + 1;
    }

    constexpr uint8_t start_data[] = {MOSI_SOF, 0x31, 0xCE};

    ExitOnFail(Prog_API::spi_transfer(start_data, nullptr, sizeof(start_data)));

    ExitOnFail(get_ack());

    uint8_t addr_data[5] = 
    {
        static_cast<uint8_t>((addr >> 24) & 0xFF),
        static_cast<uint8_t>((addr >> 16) & 0xFF),
        static_cast<uint8_t>((addr >> 8) & 0xFF),
        static_cast<uint8_t>(addr & 0xFF)
    };

    addr_data[4] = addr_data[0] ^ addr_data[1] ^ addr_data[2] ^ addr_data[3];

    ExitOnFail(Prog_API::spi_transfer(addr_data, nullptr, sizeof(addr_data)));

    ExitOnFail(get_ack());

    // maximum required length is (1 byte for length) + (up to 256 bytes data) + (1 byte checksum) = 258 bytes
    uint8_t mosi_data[258];
    memset(mosi_data, 0xFF, sizeof(mosi_data));

    mosi_data[0] = (byte_count_to_write - 1); // first byte is 1 less than the number of bytes to write

    // next len bytes are the data
    memcpy(mosi_data + 1, data, len);

    // last byte is checksum
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; ++i)
    {
        checksum = (checksum ^ data[i]);
    }
    mosi_data[byte_count_to_write + 1] = checksum;

    // number of bytes to put on bus is (1 byte for data length) + (byte count) + (1 byte for checksum)
    ExitOnFail(Prog_API::spi_transfer(mosi_data, nullptr, byte_count_to_write + 2));

    ExitOnFail(get_ack());
    return true;
}
} // namespace Boot_API
