#include "AMT242AV_Sim.h"
#include "core/VirtualClock.h"
#include "hal_mock/Arduino.h"
#include <algorithm>

namespace toad::sim {

AMT242AV_Sim::AMT242AV_Sim(uint8_t id, std::string name, size_t queue_capacity)
    : name_(std::move(name)), id_(id), queue_capacity_(queue_capacity) {}

AMT242AV_Sim::~AMT242AV_Sim() {
    stop();
}

void AMT242AV_Sim::start(int priority) {
    if (running_) return;
    running_ = true;
    rx_channel_ = std::make_unique<boost::fibers::buffered_channel<std::vector<uint8_t>>>(queue_capacity_);
    fiber_ = launch_fiber_with_priority(priority, [this]() {
        this->fiber_loop();
    });
}

void AMT242AV_Sim::stop() {
    if (!running_) return;
    running_ = false;
    if (rx_channel_) {
        rx_channel_->close();
    }
    if (fiber_.joinable()) {
        fiber_.join();
    }
}

void AMT242AV_Sim::join() {
    if (fiber_.joinable()) {
        fiber_.join();
    }
}

void AMT242AV_Sim::set_position_fraction(float fraction) {
    fraction = std::max(0.0f, std::min(1.0f, fraction));
    raw_pos_ = static_cast<uint16_t>(fraction * 4095.0f);
}

float AMT242AV_Sim::get_position_fraction() const {
    return static_cast<float>(raw_pos_) / 4095.0f;
}

void AMT242AV_Sim::set_raw_position(uint16_t pos12) {
    raw_pos_ = pos12 & 0x0FFF;
}

uint8_t AMT242AV_Sim::calculate_checksum(uint16_t pos12) {
    uint8_t cs = 0;
    // Highest bit is for odd-numbered bits, second highest is for even
    for (int i = 0; i < 6; ++i) {
        cs ^= (pos12 >> (i * 2));
    }
    // Odd parity
    return (~cs) & 0b11;
}

void AMT242AV_Sim::on_receive_bytes(const uint8_t* buffer, size_t size) {
    if (!buffer || size == 0) return;

    if (running_ && rx_channel_) {
        rx_channel_->push(std::vector<uint8_t>(buffer, buffer + size));
    } else {
        process_packet(buffer, size);
    }
}

void AMT242AV_Sim::fiber_loop() {
    while (running_) {
        std::vector<uint8_t> packet;
        boost::fibers::channel_op_status status = rx_channel_->pop(packet);
        if (status == boost::fibers::channel_op_status::closed || !running_) {
            break;
        }
        if (packet.empty()) continue;

        if (response_delay_us_ > 0) {
            VirtualClock::instance().sleep_for(response_delay_us_);
        }

        process_packet(packet.data(), packet.size());
    }
}

void AMT242AV_Sim::process_packet(const uint8_t* buffer, size_t size) {
    if (!buffer || size == 0) return;

    uint8_t cmd = buffer[0];

    if (cmd == id_) {
        // Read position command
        uint8_t cs = calculate_checksum(raw_pos_);
        // Format according to AMT24 datasheet:
        // bits 0..1: undefined/throwaway (0b00)
        // bits 2..13: 12-bit position data
        // bits 14..15: 2-bit checksum
        uint16_t response = (static_cast<uint16_t>(cs) << 14) | ((raw_pos_ & 0x0FFF) << 2);

        uint8_t reply[2];
        reply[0] = static_cast<uint8_t>(response & 0xFF);         // Low byte first
        reply[1] = static_cast<uint8_t>((response >> 8) & 0xFF);  // High byte second

        transmit_to_bus(reply, 2);
    } else if (cmd == (id_ | 0x02)) {
        // Zero position command (sets zero reference point)
        raw_pos_ = 0;
    } else if (cmd == (id_ | 0x03)) {
        // Reset command: performs a reset of the controller (does not zero the absolute position)
        reset_count_++;
        if (reset_delay_us_ > 0) {
            VirtualClock::instance().sleep_for(reset_delay_us_);
        }
    }
}

} // namespace toad::sim
