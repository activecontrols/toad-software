#pragma once

#include <cstdint>
#include <string>
#include <memory>

#include "api/HardwareCAN.h"
#include "api/CanMsg.h"

namespace toad::sim {
class CANBus;
}

/**
 * @brief Concrete embedded CAN controller class inheriting from Arduino HardwareCAN.
 *
 * Implements the standard embedded Arduino CAN driver interface.
 * Forwards physical transmissions down to the underlying simulated CANBus fabric.
 */
class CAN : public arduino::HardwareCAN {
public:
    CAN(uint32_t rx_pin = 0, uint32_t tx_pin = 0);
    explicit CAN(std::string bus_name);
    ~CAN() override = default;

    CAN(const CAN&) = default;
    CAN& operator=(const CAN&) = default;

    // HardwareCAN interface implementation
    bool begin(CanBitRate const can_bitrate) override;
    void end() override;

    int write(const arduino::CanMsg& msg) override;
    size_t available() override;
    arduino::CanMsg read() override;

    // Additional embedded convenience overloads
    bool begin(unsigned long baud = 500000);
    int write(uint32_t id, const uint8_t* data, uint8_t len, bool extended = false);

    void set_filter(uint32_t id, uint32_t mask = 0x7FF);

    uint32_t rx_pin() const { return rx_pin_; }
    uint32_t tx_pin() const { return tx_pin_; }

    void attach_bus(std::shared_ptr<toad::sim::CANBus> bus);
    std::shared_ptr<toad::sim::CANBus> get_bus() const;

private:
    void resolve_bus();

    uint32_t rx_pin_{0};
    uint32_t tx_pin_{0};
    std::string bus_name_;
    std::shared_ptr<toad::sim::CANBus> backend_bus_{nullptr};
    uint32_t filter_id_{0};
    uint32_t filter_mask_{0};
    bool filter_enabled_{false};
};
namespace arduino {
using ::CanBitRate;
using ::CAN;
}