#include "CAN.h"
#include "bus/CANBus.h"
#include "core/BusRegistry.h"

CANClass::CANClass(uint32_t rx_pin, uint32_t tx_pin)
    : rx_pin_(rx_pin), tx_pin_(tx_pin) {}

CANClass::CANClass(std::string bus_name)
    : bus_name_(std::move(bus_name)) {}

void CANClass::resolve_bus() {
    if (backend_bus_) return;

    if (!bus_name_.empty()) {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_named_can(bus_name_);
    } else if (rx_pin_ != 0 || tx_pin_ != 0) {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_can(rx_pin_, tx_pin_);
    } else {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_named_can("CAN_DEFAULT");
    }
}

bool CANClass::begin(uint32_t baud_rate) {
    resolve_bus();
    if (backend_bus_) {
        backend_bus_->set_bitrate(baud_rate);
        return true;
    }
    return false;
}

void CANClass::end() {
    // Keep connection for inspection
}

bool CANClass::write(const toad::sim::CanFrame& frame) {
    resolve_bus();
    if (!backend_bus_) return false;
    return backend_bus_->transmit_from_firmware(frame);
}

bool CANClass::write(uint32_t id, const uint8_t* data, uint8_t len, bool extended, bool rtr) {
    toad::sim::CanFrame frame(id, data, len, extended, rtr);
    return write(frame);
}

bool CANClass::read(toad::sim::CanFrame& frame) {
    resolve_bus();
    if (!backend_bus_) return false;

    while (backend_bus_->read_to_firmware(frame)) {
        if (!filter_enabled_ || (frame.id & filter_mask_) == (filter_id_ & filter_mask_)) {
            return true;
        }
        // Dropped frame that didn't match filter
    }
    return false;
}

int CANClass::available() {
    resolve_bus();
    if (!backend_bus_) return 0;
    return backend_bus_->firmware_available();
}

void CANClass::set_filter(uint32_t id, uint32_t mask) {
    filter_id_ = id;
    filter_mask_ = mask;
    filter_enabled_ = true;
}

std::shared_ptr<toad::sim::CANBus> CANClass::backend_bus() const {
    const_cast<CANClass*>(this)->resolve_bus();
    return backend_bus_;
}

// Global default instances
CANClass CAN_TVC("CAN_TVC");
CANClass CAN_FC("CAN_FC");
CANClass CAN("CAN_DEFAULT");

