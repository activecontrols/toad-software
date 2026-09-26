#pragma once
#include "GenericPeripheral.h"
#include "GenericBus.h"
#include "UartBus.h"
#include "api/ArduinoAPI.h"

class UartPeripheral : public GenericPeripheral<UartMessage>
{
private:
    UART_CHANNEL tx_ch;
public:
    UartPeripheral(UART_CHANNEL tx_ch_, std::string name_) : tx_ch(tx_ch_), GenericPeripheral<UartMessage>(name_) {}
};