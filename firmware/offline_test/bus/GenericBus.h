#pragma once

#include <string>
#include <vector>
#include <memory>
#include "GenericPeripheral.h"
#include <stdexcept>
#include <boost/fiber/all.hpp>
#include <mutex>

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
    size_t transfer(MessageType& msg)
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
            if (peripheral_ptr->receive_data(msg))
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