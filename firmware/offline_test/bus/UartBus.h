#pragma once

#include "GenericBus.h"
#include <vector>
#include <string>
#include "api/Common.h"

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

public:
    UartBus(pin_size_t rx_, pin_size_t tx_, std::string name) : rx(rx_), tx(tx_), GenericBus<UartMessage>(name) {}

    void begin(void)
    {
        // TODO register bus in the registry
    }
};