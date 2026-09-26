#pragma once

#include "GenericPeripheral.h"
#include "GenericBus.h"
#include "SPIBus.h"
#include "api/Common.h"
#include "SimulatedGPIO.h"

class SPIPeripheral : public GenericPeripheral<SPIMessage>
{
private:
    pin_size_t cs;

public:
    SPIPeripheral(pin_size_t cs_, std::string name_) : cs(cs_), GenericPeripheral<SPIMessage>(name_) {}

    bool is_selected(void)
    {
        if (toad::sim::SimulatedGPIO::instance().read_pin(cs) == LOW)
        {
            return true;
        }
        return false;
    }
};