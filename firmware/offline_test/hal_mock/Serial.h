#pragma once

#include <cstdint>
#include <memory>
#include "api/HardwareSerial.h"

namespace toad::sim {
class UartBus;
}

class Uart : public arduino::HardwareSerial {
public:
    Uart(uint32_t rx = 0, uint32_t tx = 0, uint32_t de = 0);
    virtual ~Uart() = default;

    void begin(unsigned long baud) override;
    void begin(unsigned long baud, uint16_t config) override;
    void end() override;

    size_t write(uint8_t byte) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    using arduino::Print::write; // Bring in write(const char*)

    int read() override;
    int available() override;
    int peek() override;
    void flush() override;

    operator bool() override { return true; }

    uint32_t rx_pin() const { return rx_; }
    uint32_t tx_pin() const { return tx_; }
    uint32_t de_pin() const { return de_; }

    void attach_bus(std::shared_ptr<toad::sim::UartBus> bus);
    std::shared_ptr<toad::sim::UartBus> get_bus() const;

private:
    void resolve_bus();

    uint32_t rx_;
    uint32_t tx_;
    uint32_t de_;
    std::shared_ptr<toad::sim::UartBus> backend_bus_{nullptr};
};

// Emulate USB Serial for Arduino / Teensy
class USBSerial : public Uart {
public:
    USBSerial() : Uart(0, 0, 0) {}
};

// Default Arduino Serial instance
extern Uart Serial;
