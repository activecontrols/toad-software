#include "MksServo57D_Sim.h"
#include "bus/CANBus.h"
#include <cmath>
#include <algorithm>

namespace toad::sim {

uint8_t MksServo57D_Sim::calculate_checksum(uint16_t can_id, const uint8_t* data, size_t len_without_crc) {
    uint8_t crc = static_cast<uint8_t>(can_id);
    for (size_t i = 0; i < len_without_crc; ++i) {
        crc += data[i];
    }
    return crc;
}

MksServo57D_Sim::MksServo57D_Sim(std::string name, uint32_t can_id)
    : name_(std::move(name)), can_id_(can_id) {}

MksServo57D_Sim::~MksServo57D_Sim() {
    stop();
}

void MksServo57D_Sim::enqueue_frame(const CanFrame& frame) {
    if (!accepts_frame(frame)) return;

    if (running_ && rx_channel_) {
        rx_channel_->push(frame);
    } else {
        process_frame(frame);
    }
}

void MksServo57D_Sim::start(int priority) {
    if (running_) return;
    running_ = true;
    rx_channel_ = std::make_unique<boost::fibers::buffered_channel<CanFrame>>(queue_capacity_);

    fiber_ = launch_fiber_with_priority(priority, [this]() {
        this->fiber_loop();
    });
}

void MksServo57D_Sim::stop() {
    if (!running_) return;
    running_ = false;
    VirtualClock::instance().wake_all();
    if (rx_channel_) {
        rx_channel_->close();
    }
    if (fiber_.joinable()) {
        fiber_.join();
    }
}

void MksServo57D_Sim::join() {
    if (fiber_.joinable()) {
        fiber_.join();
    }
}

void MksServo57D_Sim::fiber_loop() {
    while (running_) {
        // 1. Drain all pending incoming command frames without blocking
        if (rx_channel_) {
            CanFrame frame;
            while (rx_channel_->try_pop(frame) == boost::fibers::channel_op_status::success) {
                process_frame(frame);
            }
        }

        // 2. Advance physics by one timestep
        float dt_sec = static_cast<float>(timestep_us_) / 1000000.0f;
        update_physics(dt_sec);

        // 3. Sleep until next integration step
        VirtualClock::instance().sleep_for(timestep_us_);
    }
}

void MksServo57D_Sim::process_frame(const CanFrame& frame) {
    if (frame.len < 2) return;

    // Checksum byte is at the end of the frame: frame.data[frame.len - 1]
    uint8_t expected_crc = calculate_checksum(can_id_, frame.data, frame.len - 1);
    if (frame.data[frame.len - 1] != expected_crc) {
        crc_error_count_++;
        return;
    }

    uint8_t cmd = frame.data[0];

    // Command 0xF6: Speed & Acceleration Command
    // byte 0: 0xF6
    // byte 1: [dir:1][unused:3][spd_high:4]
    // byte 2: [spd_low:8]
    // byte 3: [acceleration:8]
    // byte 4: [crc:8]
    if (cmd == 0xF6 && frame.len >= 5) {
        bool dir = (frame.data[1] & 0x80) != 0;
        uint16_t raw_spd = static_cast<uint16_t>(((frame.data[1] & 0x0F) << 8) | frame.data[2]);
        raw_spd = std::min<uint16_t>(raw_spd, 400); // 400 RPM max per datasheet

        target_speed_rpm_ = dir ? static_cast<int16_t>(raw_spd) : -static_cast<int16_t>(raw_spd);
        acceleration_ = frame.data[3];
        command_count_++;
    }
    // Command 0x30 / 0x36: Read status / position
    else if (cmd == 0x30 || cmd == 0x36) {
        command_count_++;
        transmit_telemetry();
    }
}

void MksServo57D_Sim::update_physics(float dt_sec) {
    if (dt_sec <= 0.0f) return;

    // Acceleration rate: ~100 RPM/sec per acceleration unit
    float max_delta = (acceleration_ * 100.0f) * dt_sec;
    float speed_diff = static_cast<float>(target_speed_rpm_) - current_speed_rpm_;

    if (std::abs(speed_diff) <= max_delta) {
        current_speed_rpm_ = static_cast<float>(target_speed_rpm_);
    } else if (speed_diff > 0.0f) {
        current_speed_rpm_ += max_delta;
    } else {
        current_speed_rpm_ -= max_delta;
    }

    // Integrate angular position: (RPM / 60) * 360 deg/sec = RPM * 6 deg/sec
    float d_angle = (current_speed_rpm_ * 6.0f) * dt_sec;
    current_angle_deg_ += d_angle;
}

void MksServo57D_Sim::transmit_telemetry() {
    auto b = bus_.lock();
    if (!b) return;

    CanFrame tx_frame;
    tx_frame.id = can_id_;
    tx_frame.extended = false;
    tx_frame.rtr = false;
    tx_frame.len = 6;

    int16_t spd = static_cast<int16_t>(current_speed_rpm_);
    int16_t angle_fixed = static_cast<int16_t>(current_angle_deg_ * 10.0f); // 0.1 deg resolution

    tx_frame.data[0] = 0x31; // Status reply command
    tx_frame.data[1] = static_cast<uint8_t>((spd >> 8) & 0xFF);
    tx_frame.data[2] = static_cast<uint8_t>(spd & 0xFF);
    tx_frame.data[3] = static_cast<uint8_t>((angle_fixed >> 8) & 0xFF);
    tx_frame.data[4] = static_cast<uint8_t>(angle_fixed & 0xFF);
    tx_frame.data[5] = calculate_checksum(can_id_, tx_frame.data, 5);

    b->broadcast(tx_frame, this);
}

} // namespace toad::sim

