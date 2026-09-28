#pragma once
#include "GenericBus.h"
#include "UartBus.h"
#include "api/ArduinoAPI.h"

class UartPeripheral : public GenericPeripheral<UartMessage>
{
private:
    UART_CHANNEL tx_ch;
public:
    UartPeripheral(UART_CHANNEL tx_ch_, std::string name_) : tx_ch(tx_ch_), GenericPeripheral<UartMessage>(name_) {}

    bool observe_data(UartMessage& data) override
    {
        if (data.channel != tx_ch)
        {
            // only list to data not transmitted on our transmit channel (therefore it's on our rx channel)
            receive_data(data);
            return true;
        }
        return false;
    }

    void transfer(std::vector<uint8_t> &data)
    {
        UartMessage msg = 
        {
            .channel = tx_ch,
            .data = data
        };

        GenericPeripheral<UartMessage>::transfer(msg);
    }
};