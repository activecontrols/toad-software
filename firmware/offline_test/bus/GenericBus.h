#pragma once

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <boost/fiber/all.hpp>
#include <mutex>
#include "VirtualClock.h"
#include "FiberScheduler.h"


template <typename MessageType>
class GenericBus;

// a generic peripheral type
// a peripheral can be attached to a bus and can receive/transmit data on the bus
// peripherals are exposed to all data transacted on the bus and are responsible for filtering the data
template <typename MessageType>
class GenericPeripheral
{
private:
    
    std::atomic<bool> running = false;
    std::shared_ptr<GenericBus<MessageType>> bus;

    boost::fibers::fiber my_fiber;
    std::string name;

public:
    GenericPeripheral(std::string name_) : name(name_) {}

    virtual bool receive_data(MessageType& data) = 0;

    // by default, the peripheral always "observes" the data
    // this method can be overridden such that only when a device is selected does it "receive" the data
    // returns true when the data is "received" by the device
    virtual bool observe_data(MessageType& data) {receive_data(data); return true;}

    void attachBus(std::shared_ptr<GenericBus<MessageType>> bus_)
    {
        bus = bus_;
    }


    virtual void update_loop(void)
    {
        VirtualClock::instance().sleep_for(10000);
    }

    virtual void transfer(MessageType& data)
    {
        bus->beginTransaction();
        bus->transfer(data);
        bus->endTransaction();
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


// generic Bus type
// a bus connects any number of devices together
// all devices on a bus are able to see the data transmitted by the bus
// additionally, up to one attached device is allowed to send data back during a transaction
template <typename MessageType>
class GenericBus
{
private:
    std::string name;
    
    std::vector<std::shared_ptr<GenericPeripheral<MessageType>>> peripherals;
    int reply_device = -1;
    bool isTransacting = false;
    boost::fibers::mutex mtx_ = boost::fibers::mutex();

public:
    GenericBus(std::string name_) : name(name_) {}

    void beginTransaction(void)
    {
        std::lock_guard<boost::fibers::mutex> lock(mtx_);
        if (isTransacting)
        {
            throw std::runtime_error("Error: bus contention on " + name);
        }
        isTransacting = true;
        reply_device = -1;
    }

    void endTransaction(void)
    {
        std::lock_guard<boost::fibers::mutex> lock(mtx_);
        isTransacting = false;
    }

    // this must be called prior to starting the simulation since it is not mutex protected
    void attachPeripheral(std::shared_ptr<GenericPeripheral<MessageType>> peripheral)
    {
        if (isTransacting)
        {
            throw std::runtime_error("Error: device attached to bus while transaction in progress.");
        }
        peripherals.push_back(peripheral);
    }


    // returns the number of devices that "accepted" the message (that didn't ignore it)
    virtual size_t transfer(MessageType& msg)
    {
        std::lock_guard<boost::fibers::mutex> lock(mtx_);
        if (!isTransacting)
        {
            throw std::runtime_error("Error: bus transfer() called while a transfer is not in progress on " + name);
        }

        size_t accept_count {0};

        // forward data to all attached peripherals
        // (they can decide whether to listen to it or not)
        for (auto peripheral_ptr : peripherals)
        {
            if (peripheral_ptr->observe_data(msg))
            {
                ++accept_count;
            }
        }

        return accept_count;
    }

    const std::string& getName(void)
    {
        return this->name;
    }
};