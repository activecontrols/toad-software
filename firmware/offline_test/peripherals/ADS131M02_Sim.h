#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <mutex>
#include <memory>

#include <boost/fiber/fiber.hpp>
#include "ISPIDevice.h"
#include "hal_mock/pins_arduino.h"
#include "core/FiberScheduler.h"

namespace toad::sim {

/**
 * @brief Behavioral model of the Texas Instruments ADS131M02 24-bit simultaneous-sampling ADC.
 *
 * Supports:
 * - Independent background fiber conversion loop running at PRIO_SENSORS (default 1 kHz / 1000 us).
 * - Full-duplex synchronous SPI shift register transfers (4-word / 12-byte frames).
 * - 24-bit signed channel readings (CH0 and CH1).
 * - Real-time CCITT-CRC16 generation matching firmware ADS131M02::read_adc().
 * - DRDY pin interrupt pulsing via SimulatedGPIO.
 */
class ADS131M02_Sim : public ISPIDevice {
public:
    explicit ADS131M02_Sim(std::string name = "ADS131M02_ADC", uint32_t drdy_pin = NC);
    ~ADS131M02_Sim() override;

    const std::string& name() const override { return name_; }

    // --- Fiber Lifecycle (Concurrent Background Conversion) ---
    void start(int priority = PRIO_SENSORS) override;
    void stop() override;
    void join() override;
    bool is_running() const override { return running_; }

    // --- Channel Readings Configuration ---
    void set_ch0_raw(int32_t raw_val);
    int32_t get_ch0_raw() const;

    void set_ch1_raw(int32_t raw_val);
    int32_t get_ch1_raw() const;

    void set_status_reg(uint32_t status);
    uint32_t get_status_reg() const;

    // --- Conversion Timing & DRDY Configuration ---
    void set_sample_period_us(uint32_t period_us);
    uint32_t sample_period_us() const;

    void set_drdy_pin(uint32_t pin);
    uint32_t drdy_pin() const;

    // --- ISPIDevice Interface ---
    void on_cs_asserted() override;
    void on_cs_deasserted() override;
    uint8_t transfer_byte(uint8_t mosi_byte) override;

    // Helper: calculate CCITT-CRC16 over buffer (matches ADS131M02.cpp)
    static uint16_t calculate_crc(const uint8_t* data, size_t len);

private:
    void conversion_loop();
    void update_latched_frame();

    std::string name_;
    uint32_t drdy_pin_{NC};
    uint32_t sample_period_us_{1000}; // Default 1 kHz (1000 us) ADC conversion period

    int32_t ch0_raw_{0};
    int32_t ch1_raw_{0};
    uint32_t status_reg_{0x220000}; // Default ADS131M02 status word

    mutable std::mutex frame_mtx_;
    std::array<uint8_t, 12> latched_frame_{};
    size_t byte_idx_{0};

    bool running_{false};
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim

