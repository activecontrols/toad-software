#include "SPI.h"
#include "core/BusRegistry.h"
#include "bus/SPIBus.h"
#include "hardware_mapping/ec_pins.h"

SPIClass::SPIClass(uint32_t mosi, uint32_t miso, uint32_t sck)
    : mosi_(mosi), miso_(miso), sck_(sck) {}

void SPIClass::resolve_bus() {
    if (!backend_bus_) {
        backend_bus_ = toad::sim::BusRegistry::instance().get_or_create_spi(mosi_, miso_, sck_);
    }
}

void SPIClass::attach_bus(std::shared_ptr<toad::sim::SPIBus> bus) {
    backend_bus_ = bus;
}

std::shared_ptr<toad::sim::SPIBus> SPIClass::get_bus() const {
    const_cast<SPIClass*>(this)->resolve_bus();
    return backend_bus_;
}

void SPIClass::begin() {
    resolve_bus();
}

void SPIClass::end() {
}

void SPIClass::beginTransaction(arduino::SPISettings settings) {
    resolve_bus();
    if (backend_bus_) {
        backend_bus_->begin_transaction(settings);
    }
}

void SPIClass::endTransaction() {
    resolve_bus();
    if (backend_bus_) {
        backend_bus_->end_transaction();
    }
}

uint8_t SPIClass::transfer(uint8_t data) {
    resolve_bus();
    if (backend_bus_) {
        return backend_bus_->transfer(data);
    }
    return 0xFF;
}

uint16_t SPIClass::transfer16(uint16_t data) {
    resolve_bus();
    if (backend_bus_) {
        return backend_bus_->transfer16(data);
    }
    return 0xFFFF;
}

void SPIClass::transfer(void *buf, size_t count) {
    resolve_bus();
    if (backend_bus_ && buf && count > 0) {
        backend_bus_->transfer(buf, count);
    }
}

void SPIClass::usingInterrupt(int interruptNumber) { (void)interruptNumber; }
void SPIClass::notUsingInterrupt(int interruptNumber) { (void)interruptNumber; }
void SPIClass::attachInterrupt() {}
void SPIClass::detachInterrupt() {}

// Weak default instances allowing test binaries to link without ec_main.cpp
__attribute__((weak)) SPIClass PT_TC_SPI_1(PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK);
__attribute__((weak)) SPIClass PT_TC_SPI_3(PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK);

