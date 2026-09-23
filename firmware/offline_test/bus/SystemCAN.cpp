#include "SystemCAN.h"

namespace toad::sim {

SystemCAN::SystemCAN(std::string interface_name, uint32_t bitrate)
    : interface_name_(std::move(interface_name)), bitrate_(bitrate) {
    local_bus_ = std::make_shared<CANBus>(interface_name_, bitrate_);
}

bool SystemCAN::open() {
    is_open_ = true;
    return true;
}

void SystemCAN::close() {
    is_open_ = false;
}

bool SystemCAN::send(const CanFrame& frame) {
    if (!is_open_) return false;
    return local_bus_->transmit_from_firmware(frame);
}

bool SystemCAN::receive(CanFrame& frame) {
    if (!is_open_) return false;
    return local_bus_->read_to_firmware(frame);
}

int SystemCAN::available() {
    if (!is_open_) return 0;
    return local_bus_->firmware_available();
}

} // namespace toad::sim

