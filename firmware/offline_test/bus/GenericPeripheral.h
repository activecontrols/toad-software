#pragma once

#include "GenericBus.h"
#include <vector>
#include <string>
#include <stdint.h>
#include "FiberScheduler.h"
#include <mutex>
#include "VirtualClock.h"


// a generic peripheral type
// a peripheral can be attached to a bus and can receive/transmit data on the bus
// peripherals are exposed to all data transacted on the bus and are responsible for filtering the data
template <typename MessageType>
class GenericPeripheral
{
private:
    // bool should be atomic therefore no need to protect with mutex
    bool running = false;

    boost::fibers::fiber my_fiber;
    std::string name;

public:
    GenericPeripheral(std::string name_) : name(name_) {}

    virtual bool receive_data(MessageType& data) = 0;

    virtual void update_loop(void)
    {
        VirtualClock::instance().sleep_for(10000);
    }

    virtual void transferBegin(void) {};
    virtual void transferEnd(void) {};

    virtual void stop(void)
    {
        if (running)
        {            
            running = false;
            
            VirtualClock::instance().wake_all();

            if (my_fiber.joinable())
            {
                my_fiber.join();
            }
        }
    }

    virtual void begin(void)
    {
        if (running) throw std::runtime_error("begin() called on already running GenericPeripheral");

        running = true;
    
        my_fiber = toad::sim::launch_fiber_with_priority(toad::sim::PRIO_SENSORS, [this]()
        {
            while (running)
            {
                this->update_loop();
            }
        });
    }
};