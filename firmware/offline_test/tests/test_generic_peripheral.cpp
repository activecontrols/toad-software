#include <stdio.h>
#include "GenericBus.h"
#include "GenericPeripheral.h"
#include "FiberScheduler.h"

#include "SPIPeripheral.h"
#include "SPIBus.h"

#include "UartBus.h"
#include "UartPeripheral.h"

using namespace toad::sim;

struct SimpleMessage
{
    std::vector<uint8_t> data;
    uint32_t response;
};

class AdderPeripheral : public GenericPeripheral<SimpleMessage>
{
private:
public:
    using GenericPeripheral<SimpleMessage>::GenericPeripheral;

    bool receive_data(SimpleMessage& data) override
    {
        uint32_t sum {0};

        for (auto a : data.data)
        {
            sum += a;
        }

        data.response = sum;
        return true;
    }

    void update_loop(void) override
    {
        VirtualClock::instance().sleep_for(10000);
    }

    void transferBegin(void) override {};
    void transferEnd(void) override {};
};

void generic_peripheral_test(void)
{
    printf("begin generic peripheral test\n");


    auto adder = std::make_shared<AdderPeripheral>("adder");
    auto bus = std::make_shared<GenericBus<SimpleMessage>>("adder bus");

    bus->attachPeripheral(adder);

    SimpleMessage msg = 
    {
        .data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10},
        .response = 0
    };

    adder->begin();

    auto test_fiber = launch_fiber_with_priority(10, [&msg, &bus, &adder]()
    {
        bus->beginTransaction();
        bus->transfer(msg);
        bus->endTransaction();
    });

    test_fiber.join();

    adder->stop();

    printf("Returned value: %u\n", msg.response);

    return;
}

class SPIInverter : public SPIPeripheral
{
public:
    using SPIPeripheral::SPIPeripheral;

    bool receive_data(SPIMessage& data) override
    {
        if (!this->is_selected()) return false;
        data.miso = ~data.mosi;
        return true;
    }
};

void spi_bus_test(void)
{
    printf("begin spi bus test\n");

    SPIMessage msg = 
    {
        .mosi = 0x55,
        .miso = 0xFF
    };

    auto spi_bus = std::make_shared<SPIBus>(0, 0, 0, "test spi bus");
    auto spi_inverter = std::make_shared<SPIInverter>(1, "test spi inverter");
    
    spi_bus->attachPeripheral(spi_inverter);

    spi_inverter->begin();

    auto test_fiber = launch_fiber_with_priority(10, [&msg, &spi_bus, &spi_inverter]()
    {
        digitalWrite(1, LOW);
        pinMode(1, OUTPUT);
        spi_bus->beginTransaction();
        spi_bus->transfer(msg);
        spi_bus->endTransaction();

        digitalWrite(1, HIGH);
    });

    test_fiber.join();
    spi_inverter->stop();

    printf("Response: 0x%02hhX\n", msg.miso);
    printf("Expected response: 0x%02hhX\n", ~msg.mosi);

    return;
}

class SimpleUartReplier : public UartPeripheral 
{
public:
    using UartPeripheral::UartPeripheral;

    bool receive_data(UartMessage& msg) override
    {
        
        return true;
    }
};


void uart_bus_test(void)
{
    auto uart_bus = std::make_shared<UartBus>(1, 2, "uart bus");

    return;
}

int main(void)
{
    install_fiber_scheduler();

    generic_peripheral_test();

    spi_bus_test();

    return 0;
}