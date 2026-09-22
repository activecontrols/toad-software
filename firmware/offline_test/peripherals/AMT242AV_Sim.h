#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <vector>

#include <boost/fiber/fiber.hpp>
#include <boost/fiber/buffered_channel.hpp>
#include "IRS485Device.h"
#include "core/FiberScheduler.h"

namespace toad::sim {

/**
 * @brief Behavioral model of the Broadcom/CUI AMT242AV 12-bit modular absolute rotary shaft encoder.
 *
 * Supports:
 * - Pattern 2 asynchronous fiber queue execution via start()
 * - Synchronous fallback execution when start() is not called
 * - 12-bit angular position formatting and 2-bit odd parity checksum calculation
 * - Zero and Reset command execution
 */
class AMT242AV_Sim : public IRS485Device {
public:
    explicit AMT242AV_Sim(uint8_t id = 0x00,
                          std::string name = "AMT242AV_Encoder",
                          size_t queue_capacity = 16);
    ~AMT242AV_Sim() override;

    const std::string& name() const override { return name_; }

    // --- Fiber Lifecycle (Pattern 2) ---
    void start(int priority = PRIO_SENSORS) override;
    void stop() override;
    void join() override;
    bool is_running() const override { return running_; }

    // --- Position Controls for Tests and Simulation ---
    void set_position_fraction(float fraction);
    float get_position_fraction() const;

    void set_raw_position(uint16_t pos12);
    uint16_t get_raw_position() const { return raw_pos_; }

    uint8_t id() const { return id_; }
    void set_id(uint8_t id) { id_ = id; }

    // --- Latency & Reset Configuration ---
    uint32_t response_delay_us() const { return response_delay_us_; }
    void set_response_delay_us(uint32_t delay_us) { response_delay_us_ = delay_us; }

    uint32_t reset_delay_us() const { return reset_delay_us_; }
    void set_reset_delay_us(uint32_t delay_us) { reset_delay_us_ = delay_us; }

    size_t reset_count() const { return reset_count_; }
    void clear_reset_count() { reset_count_ = 0; }

    // Called by RS485Mux when firmware transmits to this device
    void on_receive_bytes(const uint8_t* buffer, size_t size) override;

    // Helper: calculate 2-bit odd parity checksum
    static uint8_t calculate_checksum(uint16_t pos12);

private:
    void fiber_loop();
    void process_packet(const uint8_t* buffer, size_t size);

    std::string name_;
    uint8_t id_{0x00};
    uint16_t raw_pos_{0};            // 12-bit position (0 - 4095)
    uint32_t response_delay_us_{70}; // Physical response latency (~70 us per datasheet)
    uint32_t reset_delay_us_{5000};  // Default 5 ms controller reboot delay
    size_t reset_count_{0};

    // Fiber queue (Pattern 2)
    size_t queue_capacity_{16};
    bool running_{false};
    std::unique_ptr<boost::fibers::buffered_channel<std::vector<uint8_t>>> rx_channel_{nullptr};
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim
