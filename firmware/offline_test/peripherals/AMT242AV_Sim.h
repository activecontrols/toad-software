#pragma once

#include <cstdint>
#include <string>
#include "IRS485Device.h"

namespace toad::sim {

class AMT242AV_Sim : public IRS485Device {
public:
    explicit AMT242AV_Sim(uint8_t id = 0x00, std::string name = "AMT242AV_Encoder");
    ~AMT242AV_Sim() override = default;

    const std::string& name() const override { return name_; }

    // Position controls for tests and simulation
    void set_position_fraction(float fraction);
    float get_position_fraction() const;

    void set_raw_position(uint16_t pos12);
    uint16_t get_raw_position() const { return raw_pos_; }

    uint8_t id() const { return id_; }
    void set_id(uint8_t id) { id_ = id; }

    // Controller reset state & delay
    uint32_t reset_delay_us() const { return reset_delay_us_; }
    void set_reset_delay_us(uint32_t delay_us) { reset_delay_us_ = delay_us; }

    size_t reset_count() const { return reset_count_; }
    void clear_reset_count() { reset_count_ = 0; }

    void on_receive_bytes(const uint8_t* buffer, size_t size) override;

    // Helper: calculate 2-bit odd parity checksum
    static uint8_t calculate_checksum(uint16_t pos12);

private:
    std::string name_;
    uint8_t id_{0x00};
    uint16_t raw_pos_{0}; // 12-bit position (0 - 4095)
    uint32_t reset_delay_us_{5000}; // Default 5 ms controller reboot delay
    size_t reset_count_{0};
};

} // namespace toad::sim
