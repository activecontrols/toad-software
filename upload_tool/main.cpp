#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <vector>

#include "can_bus_defs.h"
#include "util.h"
#include "prog_api.h"
#include "boot_api.h"
#include "serial.h"

Serial serial;

using namespace CAN_Programmer;

bool program(FILE* f);

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage:\nupload_tool USB <path to .bin>\nOR\nupload_tool CAN <path to .bin>" << std::endl;
        return -1;
    }

    if (strcmp(argv[1], "USB") != 0)
    {
        std::cerr << "Only USB upload is currently supported" << std::endl;
        return -1;
    }

    FILE* f = fopen(argv[2], "rb");

    if (f == nullptr)
    {
        std::cerr << "Error: '" << argv[2] << "' not found" << std::endl;
        return -1;
    }

    if (!serial.open("/dev/ttyACM0"))
    {
        std::cerr << "Failed to open serial port" << std::endl;
        fclose(f);
        return -1;
    }

    program(f);

    fclose(f);
    serial.close();
    return 0;
}

bool program(FILE* f)
{
    ExitOnFail(Prog_API::enter_bootloader());

    ExitOnFail(Boot_API::sync_frame());

    std::cout << "SYNC frame achieved!" << std::endl;

    std::cout << "Erasing bank 1" << std::endl;

    ExitOnFail(Boot_API::bank1_erase());

    std::cout << "Erased bank 1" << std::endl;

    constexpr uint32_t bank1_start = 0x08000000;
    uint32_t write_addr = bank1_start;
    uint8_t tmp[256];

    for (;;)
    {
        size_t read_count = fread(tmp, 1, sizeof(tmp), f);

        if (read_count == 0)
        {
            if (feof(f) != EOF)
            {
                std::cerr << "Got error when reading from file:" << ferror(f) << std::endl;
                return false;
            }
            break;
        }

        ExitMsgOnFail(Boot_API::write_memory(write_addr, tmp, read_count), "program(): Boot_API::write_memory() failed");

        write_addr += read_count;

        if (read_count != sizeof(tmp))
        {
            // check for eof
            if (feof(f) != EOF)
            {
                std::cerr << "Got error when reading from file:" << ferror(f) << std::endl;
                return false;
            }
            // we have reached end of file
            break;
        }
    }

    std::cout << "Program complete" << std::endl;

    return true;
}
