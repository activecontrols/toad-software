#pragma once

#include "GenericBus.h"
#include <vector>
#include <string>
#include "api/Common.h"
#include "VirtualTerminal.h"

enum class UART_CHANNEL
{
    TX, // TX pin of microcontroller
    RX  // RX pin of microcontroller
};

struct UartMessage
{
    UART_CHANNEL channel;
    std::vector<uint8_t> data;
};


class UartBus : public GenericBus<UartMessage>
{
private:
    pin_size_t rx;
    pin_size_t tx;
    toad::sim::VirtualTerminal terminal;

public:
    UartBus(pin_size_t rx_, pin_size_t tx_, std::string name_) : rx(rx_), tx(tx_), GenericBus<UartMessage>(name_){}

    void begin(void)
    {
        // TODO register bus in the registry
    }


    size_t transfer(UartMessage& msg) override
    {
        size_t res = GenericBus<UartMessage>::transfer(msg);

        terminal.write(msg.data.data(), msg.data.size());

        return res;
    }
};