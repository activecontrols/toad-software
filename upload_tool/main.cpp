#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include "can_bus_defs.h"
#include "util.h"
#include "prog_api.h"
#include "serial.h"

Serial serial;

using namespace CAN_Programmer;

constexpr uint8_t MOSI_SYNC = 0x5A;
constexpr uint8_t MISO_SYNC = 0xA5;
constexpr uint8_t NULL_BYTE = 0x00;
constexpr uint8_t ACK = 0x79;
constexpr uint8_t NACK = 0x1F;

bool sync_frame(void);
bool program_test(void);

int main(void)
{
    if (!serial.open("/dev/ttyACM0"))
    {
        std::cout << "Failed to open serial port" << std::endl;
        return -1;
    }

    program_test();

    serial.close();
    return 0;
}

bool program_test(void)
{
    ExitOnFail(Prog_API::enter_bootloader());

    ExitOnFail(sync_frame());

    
}


bool sync_frame(void)
{
    uint8_t res = 0;
    ExitOnFail(Prog_API::spi_transfer(&MOSI_SYNC, &res, sizeof(MOSI_SYNC)));

    ExitOnFail(res == MISO_SYNC);

    ExitOnFail(Prog_API::spi_transfer(&NULL_BYTE, &res, sizeof(NULL_BYTE)));

    // from AN[6] - loop until we receive either ACK or NACK
    // here I also impose max # of retries
    for (int i = 0; i < 100; ++i)
    {
        Prog_API::spi_transfer(&NULL_BYTE, &res, sizeof(NULL_BYTE));

        if (res == ACK)
        {
            return true;
        }

        if (res == NACK)
        {
            std::cerr << "Received NACK" << std::endl;
            return false;
        }

        sleep(10);
    }

    std::cerr << "sync_frame() hit max number of retries" << std::endl;

    return false;
}