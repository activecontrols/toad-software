#pragma once

#include <cstdint>
#include <memory>

// Rename ArduinoCore-API's typedef to avoid name collision with our concrete SPIClass
#define SPIClass __ArduinoCoreAPI_SPIClass_Typedef
#include "api/HardwareSPI.h"
#undef SPIClass

namespace toad::sim {
class SPIBus;
}

class SPIClass : public arduino::HardwareSPI {
public:
    explicit SPIClass(uint32_t mosi = 0, uint32_t miso = 0, uint32_t sck = 0);
    ~SPIClass() override = default;

    // Copyable (shares backend bus)
    SPIClass(const SPIClass& other) = default;
    SPIClass& operator=(const SPIClass& other) = default;

    // Full-duplex transfers
    uint8_t transfer(uint8_t data) override;
    uint16_t transfer16(uint16_t data) override;
    void transfer(void *buf, size_t count) override;

    // Transaction Management
    void beginTransaction(arduino::SPISettings settings) override;
    void endTransaction() override;

    void usingInterrupt(int interruptNumber) override;
    void notUsingInterrupt(int interruptNumber) override;
    void attachInterrupt() override;
    void detachInterrupt() override;

    void begin() override;
    void end() override;

    // Pin inspection & Backend bus connection
    uint32_t mosi_pin() const { return mosi_; }
    uint32_t miso_pin() const { return miso_; }
    uint32_t sck_pin() const { return sck_; }

    void attach_bus(std::shared_ptr<toad::sim::SPIBus> bus);
    std::shared_ptr<toad::sim::SPIBus> get_bus() const;

private:
    void resolve_bus();

    uint32_t mosi_{0};
    uint32_t miso_{0};
    uint32_t sck_{0};
    std::shared_ptr<toad::sim::SPIBus> backend_bus_{nullptr};
};

namespace arduino {
    using ::SPIClass;
}

// Pre-defined SPI bus instances for TOAD Engine Controller
extern SPIClass PT_TC_SPI_1;
extern SPIClass PT_TC_SPI_3;
