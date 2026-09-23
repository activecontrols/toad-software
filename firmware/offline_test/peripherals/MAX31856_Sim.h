#pragma once

#include <cstdint>
#include <string>
#include <mutex>
#include <boost/fiber/fiber.hpp>
#include "ISPIDevice.h"
#include "core/FiberScheduler.h"
#include "core/VirtualClock.h"

namespace toad::sim {

/**
 * @brief Behavioral simulation model for the Maxim MAX31856 thermocouple reader.
 *
 * Implements:
 * - SPI register read and write protocols matching Adafruit_MAX31856 driver.
 * - Factory default CJLF register value (0xC0) for driver initialization handshake.
 * - 24-bit linearized thermocouple temperature encoding (0.0078125 deg C resolution).
 * - Fiber concurrency lifecycle via start(priority), stop(), and join().
 */
class MAX31856_Sim : public ISPIDevice {
public:
    explicit MAX31856_Sim(std::string name = "MAX31856_TC", float temp_c = 25.0f)
        : name_(std::move(name)) {
        regs_[0x04] = 0xC0; // MAX31856_CJLF_REG default per datasheet and Adafruit_MAX31856::begin()
        set_temperature_c(temp_c);
    }

    ~MAX31856_Sim() override {
        stop();
    }

    const std::string& name() const override { return name_; }

    void set_temperature_c(float temp_c) {
        std::lock_guard<std::mutex> lock(mtx_);
        temp_c_ = temp_c;
        int32_t val = static_cast<int32_t>(temp_c_ / 0.0078125f);
        val <<= 5;
        regs_[0x0C] = static_cast<uint8_t>((val >> 16) & 0xFF);
        regs_[0x0D] = static_cast<uint8_t>((val >> 8) & 0xFF);
        regs_[0x0E] = static_cast<uint8_t>(val & 0xFF);
    }

    float get_temperature_c() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return temp_c_;
    }

    void start(int priority = PRIO_SENSORS) override {
        if (running_) return;
        running_ = true;
        fiber_ = launch_fiber_with_priority(priority, [this]() {
            while (running_) {
                VirtualClock::instance().sleep_for(100000); // 100ms periodic update
            }
        });
    }

    void stop() override {
        if (!running_) return;
        running_ = false;
        VirtualClock::instance().wake_all();
        if (fiber_.joinable()) {
            fiber_.join();
        }
    }

    void join() override {
        if (fiber_.joinable()) {
            fiber_.join();
        }
    }

    bool is_running() const override { return running_; }

    void on_cs_asserted() override {
        std::lock_guard<std::mutex> lock(mtx_);
        first_byte_ = true;
    }

    void on_cs_deasserted() override {
        std::lock_guard<std::mutex> lock(mtx_);
        first_byte_ = true;
    }

    uint8_t transfer_byte(uint8_t mosi) override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (first_byte_) {
            first_byte_ = false;
            if ((mosi & 0x80) != 0) {
                is_read_ = false;
                curr_addr_ = mosi & 0x0F;
            } else {
                is_read_ = true;
                curr_addr_ = mosi & 0x0F;
            }
            return 0x00;
        } else {
            if (is_read_) {
                uint8_t val = (curr_addr_ < 16) ? regs_[curr_addr_++] : 0xFF;
                return val;
            } else {
                if (curr_addr_ < 16) {
                    regs_[curr_addr_++] = mosi;
                }
                return 0x00;
            }
        }
    }

private:
    std::string name_;
    mutable std::mutex mtx_;
    uint8_t regs_[16]{0};
    float temp_c_{25.0f};
    bool first_byte_{true};
    bool is_read_{false};
    uint8_t curr_addr_{0};
    bool running_{false};
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim

