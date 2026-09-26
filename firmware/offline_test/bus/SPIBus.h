#pragma once

#include "SPIPeripheral.h"
#include "SPIBus.h"
#include "GenericBus.h"
#include "api/Common.h"

struct SPIMessage
{
    uint8_t mosi;
    uint8_t miso;
};


class SPIBus : public GenericBus<SPIMessage>
{
private:
    pin_size_t mosi;
    pin_size_t miso;
    pin_size_t sclk;

public:

    SPIBus(pin_size_t mosi_, pin_size_t miso_, pin_size_t sclk_, std::string name_) : GenericBus<SPIMessage>(name_), mosi(mosi_), miso(miso_), sclk(sclk_) {}

    void begin(void)
    {
        // TODO register bus in registry
    }

    size_t transfer(SPIMessage& msg)
    {
        size_t count = GenericBus<SPIMessage>::transfer(msg);

        // SPI bus has the restriction that there must be one device accepting the message
        // more devices means there was bus contention, no devices means a CS signal isn't connected
        if (count == 0)
        {
            throw std::runtime_error("No devices accepted SPI transfer on " + this->getName());
        }
        else if (count > 1)
        {
            throw std::runtime_error("More than one device accepted SPI transfer on " + this->getName());
        }

        return count;
    }
};