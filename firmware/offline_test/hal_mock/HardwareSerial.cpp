#include "HardwareSerial.h"
#include "core/BusRegistry.h"
#include "bus/UartBus.h"

Uart::Uart(uint32_t rx, uint32_t tx, uint32_t de)
    : rx_(rx), tx_(tx), de_(de) {}

void Uart::resolve_bus() {
    if (!backend_bus_) {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_uart(rx_, tx_, de_);
    }
}

void Uart::attach_bus(std::shared_ptr<toad::sim::UartBus> bus) {
    backend_bus_ = bus;
}

std::shared_ptr<toad::sim::UartBus> Uart::get_bus() const {
    const_cast<Uart*>(this)->resolve_bus();
    return backend_bus_;
}

void Uart::begin(unsigned long baud) {
    resolve_bus();
    if (backend_bus_) {
        backend_bus_->set_baud_rate(baud);
    }
}

void Uart::begin(unsigned long baud, uint16_t config) {
    (void)config;
    begin(baud);
}

void Uart::end() {
    // End serial session
}

size_t Uart::write(uint8_t byte) {
    resolve_bus();
    if (backend_bus_) {
        backend_bus_->write_from_firmware(byte);
        return 1;
    }
    return 0;
}

size_t Uart::write(const uint8_t *buffer, size_t size) {
    resolve_bus();
    if (backend_bus_ && buffer && size > 0) {
        backend_bus_->write_from_firmware(buffer, size);
        return size;
    }
    return 0;
}

int Uart::read() {
    resolve_bus();
    if (backend_bus_) {
        return backend_bus_->read_for_firmware();
    }
    return -1;
}

int Uart::available() {
    resolve_bus();
    if (backend_bus_) {
        return static_cast<int>(backend_bus_->available_for_firmware());
    }
    return 0;
}

int Uart::peek() {
    resolve_bus();
    if (backend_bus_) {
        return backend_bus_->peek_for_firmware();
    }
    return -1;
}

void Uart::flush() {
    resolve_bus();
    if (backend_bus_) {
        backend_bus_->flush_firmware();
    }
}

// Default Arduino Serial instance (e.g. Pin 0 RX, Pin 1 TX)
Uart Serial(0, 1, 0);

