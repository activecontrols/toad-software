#include "CAN.h"
#include "bus/CANBus.h"
#include "core/BusRegistry.h"

CAN::CAN(uint32_t rx_pin, uint32_t tx_pin)
    : rx_pin_(rx_pin), tx_pin_(tx_pin) {}

CAN::CAN(std::string bus_name)
    : bus_name_(std::move(bus_name)) {}

void CAN::resolve_bus() {
    if (backend_bus_) return;

    if (!bus_name_.empty()) {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_named_can(bus_name_);
    } else if (rx_pin_ != 0 || tx_pin_ != 0) {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_can(rx_pin_, tx_pin_);
    } else {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_named_can("CAN_DEFAULT");
    }
}

bool CAN::begin(CanBitRate const can_bitrate) {
    return begin(static_cast<unsigned long>(can_bitrate));
}


bool CAN::begin(unsigned long baud) {
    resolve_bus();
    if (backend_bus_) {
        backend_bus_->set_bitrate(baud);
        return true;
    }
    return false;
}

void CAN::end() {
    // Keep backend bus alive for inspection
}

int CAN::write(const arduino::CanMsg& msg) {
    resolve_bus();
    if (!backend_bus_) return -1;
    bool ok = backend_bus_->transmit_from_firmware(msg);
    return ok ? 1 : -1;
}

int CAN::write(uint32_t id, const uint8_t* data, uint8_t len, bool extended) {
    uint32_t msg_id = extended ? (id | arduino::CanMsg::CAN_EFF_FLAG) : (id & arduino::CanMsg::CAN_SFF_MASK);
    arduino::CanMsg msg(msg_id, len, data);
    return write(msg);
}

size_t CAN::available() {
    resolve_bus();
    if (!backend_bus_) return 0;
    return static_cast<size_t>(backend_bus_->firmware_available());
}

arduino::CanMsg CAN::read() {
    resolve_bus();
    if (!backend_bus_) return arduino::CanMsg();

    arduino::CanMsg msg;
    while (backend_bus_->read_to_firmware(msg)) {
        if (!filter_enabled_) {
            return msg;
        }
        uint32_t raw_id = msg.isExtendedId() ? msg.getExtendedId() : msg.getStandardId();
        if ((raw_id & filter_mask_) == (filter_id_ & filter_mask_)) {
            return msg;
        }
        // Dropped frame that did not match filter
    }
    return arduino::CanMsg();
}

void CAN::set_filter(uint32_t id, uint32_t mask) {
    filter_id_ = id;
    filter_mask_ = mask;
    filter_enabled_ = true;
}

void CAN::attach_bus(std::shared_ptr<toad::sim::CANBus> bus) {
    backend_bus_ = bus;
}

std::shared_ptr<toad::sim::CANBus> CAN::get_bus() const {
    const_cast<CAN*>(this)->resolve_bus();
    return backend_bus_;
}