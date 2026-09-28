#include <stdio.h>
#include "GenericBus.h"
#include "FiberScheduler.h"

#include "SPIPeripheral.h"
#include "SPIBus.h"

#include "UartBus.h"
#include "UartPeripheral.h"

#include <thread>
#include <chrono>
#include <iostream>

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

    std::vector<std::vector<uint8_t>> messages;

    boost::fibers::mutex mtx_;
    bool receive_data(UartMessage& msg) override
    {
        std::lock_guard<boost::fibers::mutex> lock(mtx_);

        messages.push_back(msg.data);

        return true;
    }

    void update_loop(void) override
    {
        VirtualClock::instance().sleep_for(100);

        {
            std::lock_guard<boost::fibers::mutex> lock(mtx_);
            if (messages.size() > 0)
            {
                std::vector<uint8_t> msg = messages[messages.size() - 1];
                // copy data to string object
                std::string s(msg.begin(), msg.end());

                printf("Peripheral: received %s\n", s.c_str());

                std::string reply_str = "hello: " + s;

                std::vector<uint8_t> reply_data = std::vector<uint8_t>(reply_str.begin(), reply_str.end());

                // put this reply on the bus
                transfer(reply_data);

                messages.erase(messages.end());
            }
        }
    }
};

class SimpleUartBuffer : public UartPeripheral
{
public:
    using UartPeripheral::UartPeripheral;

    std::vector<std::vector<uint8_t>> messages;

    bool receive_data(UartMessage& msg) override
    {
        messages.push_back(msg.data);

        return true;
    }
};


void uart_bus_test(void)
{
    auto uart_bus = std::make_shared<UartBus>(1, 2, "uart bus");
    auto uart_device = std::make_shared<SimpleUartReplier>(UART_CHANNEL::RX, "replier peripheral");
    auto uart_buffer = std::make_shared<SimpleUartBuffer>(UART_CHANNEL::TX, "uart buffer");
    uart_bus->begin();
    uart_device->begin();
    uart_buffer->begin();

    uart_bus->attachPeripheral(uart_device);
    uart_bus->attachPeripheral(uart_buffer);

    uart_device->attachBus(uart_bus);
    uart_buffer->attachBus(uart_bus);

    std::atomic<bool> test_running = true;

    auto test_fiber = launch_fiber_with_priority(10, [&uart_bus, &uart_buffer, &test_running]()
    {
        std::string message = "testing 123 from test fiber";
        auto data = std::vector<uint8_t>(message.begin(), message.end());

        UartMessage msg = 
        {
            .channel = UART_CHANNEL::TX,
            .data = data
        };

        // send data on the bus
        uart_bus->beginTransaction();
        uart_bus->transfer(msg);
        uart_bus->endTransaction();

        // receive data from the bus
        VirtualClock::instance().sleep_for(100000);

        test_running = false;
    });

    while (test_running)
    {
        VirtualClock::instance().advance_time_to(VirtualClock::instance().now_us() + 1000);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    test_fiber.join();

    printf("Message buffer length: %d\n", uart_buffer->messages.size());
    if (uart_buffer->messages.size() > 0)
    {
        printf("Message: %s\n", std::string(uart_buffer->messages[0].begin(), uart_buffer->messages[0].end()).c_str());
    }
    getchar(); // wait for user to give any input so that they can observe the bus transaction

    uart_device->stop();
    uart_buffer->stop();

    return;
}

int main(void)
{
    install_fiber_scheduler();

    generic_peripheral_test();

    spi_bus_test();

    uart_bus_test();

    return 0;
}