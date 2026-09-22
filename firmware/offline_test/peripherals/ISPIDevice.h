#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include <vector>

namespace toad::sim {

/**
 * @brief Base abstract interface for any hardware device connected to an SPI bus.
 *
 * Team members can implement this class to create simulated ADCs, thermocouple amplifiers,
 * external flash memory, or custom SPI sensors.
 */
class ISPIDevice {
public:
    virtual ~ISPIDevice() = default;

    // Device identification for debugging and transaction logs
    virtual const std::string& name() const = 0;

    // Chip select edge notifications (called when CS is asserted active or deasserted inactive)
    virtual void on_cs_asserted() {}
    virtual void on_cs_deasserted() {}

    // Full-duplex single byte transfer (MOSI in -> MISO out)
    virtual uint8_t transfer_byte(uint8_t mosi_byte) = 0;

    // Full-duplex buffer transfer (default implementation sequentially clocks transfer_byte)
    virtual void transfer_buffer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            uint8_t mosi = tx_buf ? tx_buf[i] : 0xFF;
            uint8_t miso = transfer_byte(mosi);
            if (rx_buf) {
                rx_buf[i] = miso;
            }
        }
    }
};

/**
 * @brief Lambda-based pluggable SPI device.
 *
 * Allows developers to substitute SPI sensor models or rapid test mocks in 3 lines:
 *
 * auto my_sensor = std::make_shared<FunctionalSPIDevice>("MyADC",
 *     [](uint8_t mosi, FunctionalSPIDevice& dev) -> uint8_t {
 *         // Respond with sensor data
 *         return 0x42;
 *     });
 */
class FunctionalSPIDevice : public ISPIDevice {
public:
    using ByteHandler = std::function<uint8_t(uint8_t mosi, FunctionalSPIDevice& dev)>;
    using CsHandler = std::function<void(FunctionalSPIDevice& dev)>;

    explicit FunctionalSPIDevice(std::string name, ByteHandler byte_handler)
        : name_(std::move(name)), byte_handler_(std::move(byte_handler)) {}

    FunctionalSPIDevice(std::string name,
                        ByteHandler byte_handler,
                        CsHandler on_assert,
                        CsHandler on_deassert)
        : name_(std::move(name)),
          byte_handler_(std::move(byte_handler)),
          on_assert_(std::move(on_assert)),
          on_deassert_(std::move(on_deassert)) {}

    const std::string& name() const override { return name_; }

    void on_cs_asserted() override {
        if (on_assert_) on_assert_(*this);
    }

    void on_cs_deasserted() override {
        if (on_deassert_) on_deassert_(*this);
    }

    uint8_t transfer_byte(uint8_t mosi_byte) override {
        if (byte_handler_) {
            return byte_handler_(mosi_byte, *this);
        }
        return 0xFF; // Floating MISO line default
    }

private:
    std::string name_;
    ByteHandler byte_handler_;
    CsHandler on_assert_{nullptr};
    CsHandler on_deassert_{nullptr};
};

} // namespace toad::sim

