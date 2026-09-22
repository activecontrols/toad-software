#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include <vector>

namespace toad::sim {

using DeviceTransmitFn = std::function<void(const uint8_t* buffer, size_t size)>;

/**
 * @brief Base abstract interface for any hardware device connected to an RS-485 bus.
 *
 * Team members can implement this class to create simulated sensors, motor drivers,
 * or custom hardware models.
 */
class IRS485Device {
public:
    virtual ~IRS485Device() = default;

    // Device identification
    virtual const std::string& name() const = 0;

    // Called by RS485Mux when bytes are received while this device's SEL line is active
    virtual void on_receive_bytes(const uint8_t* buffer, size_t size) = 0;

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
 * Allows developers to substitute mock models or quick test responses in a few lines of code:
 *
 * auto my_mock = std::make_shared<FunctionalRS485Device>("MyMotor",
 *     [](const uint8_t* data, size_t len, IRS485Device& dev) {
 *         // handle command and respond
 *         dev.transmit_to_bus("OK\n");
 *     });
 */
class FunctionalRS485Device : public IRS485Device {
public:
    using ReceiveHandler = std::function<void(const uint8_t* buffer, size_t size, IRS485Device& dev)>;

    FunctionalRS485Device(std::string name, ReceiveHandler handler)
        : name_(std::move(name)), handler_(std::move(handler)) {}

    const std::string& name() const override { return name_; }

    void on_receive_bytes(const uint8_t* buffer, size_t size) override {
        if (handler_) {
            handler_(buffer, size, *this);
        }
    }

private:
    std::string name_;
    ReceiveHandler handler_;
};

} // namespace toad::sim

