#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include <vector>

#include <boost/fiber/fiber.hpp>
#include <boost/fiber/buffered_channel.hpp>
#include "core/FiberScheduler.h"
#include "core/VirtualClock.h"

namespace toad::sim {

using DeviceTransmitFn = std::function<void(const uint8_t* buffer, size_t size)>;

/**
 * @brief Base abstract interface for any hardware device connected to an RS-485 bus.
 *
 * Supports both:
 * 1. Synchronous dispatch (direct callback on on_receive_bytes).
 * 2. Pattern 2 asynchronous dispatch (background fiber receiving packets via buffered_channel).
 */
class IRS485Device {
public:
    virtual ~IRS485Device() = default;

    // Device identification
    virtual const std::string& name() const = 0;

    // Called by RS485Mux when bytes are received while this device's SEL line is active
    virtual void on_receive_bytes(const uint8_t* buffer, size_t size) = 0;

    // Optional fiber lifecycle hooks (Pattern 2)
    virtual void start(int priority = PRIO_SENSORS) { (void)priority; }
    virtual void stop() {}
    virtual void join() {}
    virtual bool is_running() const { return false; }

    // Hook called by RS485Mux when device is registered
    void set_bus_transmitter(DeviceTransmitFn transmitter) {
        transmitter_ = std::move(transmitter);
    }

    // Convenience methods for devices to reply back onto the bus
    void transmit_to_bus(const uint8_t* buffer, size_t size) {
        if (transmitter_ && buffer && size > 0) {
            transmitter_(buffer, size);
        }
    }

    void transmit_to_bus(const std::string& str) {
        transmit_to_bus(reinterpret_cast<const uint8_t*>(str.data()), str.size());
    }

    void transmit_to_bus(uint8_t byte) {
        transmit_to_bus(&byte, 1);
    }

protected:
    DeviceTransmitFn transmitter_{nullptr};
};

/**
 * @brief Lambda-based pluggable RS-485 device.
 *
 * Allows developers to substitute mock models or quick test responses in a few lines of code.
 *
 * Synchronous mode (default):
 *   auto my_mock = std::make_shared<FunctionalRS485Device>("MyMotor",
 *       [](const uint8_t* data, size_t len, IRS485Device& dev) {
 *           dev.transmit_to_bus("OK\n");
 *       });
 *
 * Pattern 2 Fiber mode (asynchronous message queue):
 *   my_mock->start(PRIO_SENSORS);
 */
class FunctionalRS485Device : public IRS485Device {
public:
    using ReceiveHandler = std::function<void(const uint8_t* buffer, size_t size, IRS485Device& dev)>;

    FunctionalRS485Device(std::string name, ReceiveHandler handler, size_t queue_capacity = 16)
        : name_(std::move(name)), handler_(std::move(handler)), queue_capacity_(queue_capacity) {}

    ~FunctionalRS485Device() override {
        stop();
    }

    const std::string& name() const override { return name_; }

    void start(int priority = PRIO_SENSORS) override {
        if (running_) return;
        running_ = true;
        rx_channel_ = std::make_unique<boost::fibers::buffered_channel<std::vector<uint8_t>>>(queue_capacity_);
        fiber_ = launch_fiber_with_priority(priority, [this]() {
            while (running_) {
                std::vector<uint8_t> packet;
                boost::fibers::channel_op_status status = rx_channel_->pop(packet);
                if (status == boost::fibers::channel_op_status::closed || !running_) {
                    break;
                }
                if (handler_ && !packet.empty()) {
                    handler_(packet.data(), packet.size(), *this);
                }
            }
        });
    }

    void stop() override {
        if (!running_) return;
        running_ = false;
        VirtualClock::instance().wake_all();
        if (rx_channel_) {
            rx_channel_->close();
        }
        if (fiber_.joinable()) {
            fiber_.join();
        }
    }


    void join() override {
        if (fiber_.joinable()) {
            fiber_.join();
        }
    }

    bool is_running() const override { return running_; }

    void on_receive_bytes(const uint8_t* buffer, size_t size) override {
        if (!buffer || size == 0) return;

        if (running_ && rx_channel_) {
            rx_channel_->push(std::vector<uint8_t>(buffer, buffer + size));
        } else if (handler_) {
            handler_(buffer, size, *this);
        }
    }

private:
    std::string name_;
    ReceiveHandler handler_;
    size_t queue_capacity_{16};
    bool running_{false};
    std::unique_ptr<boost::fibers::buffered_channel<std::vector<uint8_t>>> rx_channel_{nullptr};
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim
