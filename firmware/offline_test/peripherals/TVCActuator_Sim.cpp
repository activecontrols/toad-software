#include "TVCActuator_Sim.h"
#include "bus/CANBus.h"
#include <cmath>
#include <algorithm>

namespace toad::sim {

TVCActuator_Sim::TVCActuator_Sim(std::string name, uint32_t can_id, float initial_length_mm)
    : name_(std::move(name)), can_id_(can_id),
      current_length_mm_(initial_length_mm), target_length_mm_(initial_length_mm) {}

TVCActuator_Sim::~TVCActuator_Sim() {
    stop();
}

void TVCActuator_Sim::set_target_length_mm(float target_mm) {
    target_length_mm_ = std::max(min_length_mm_, std::min(max_length_mm_, target_mm));
}

void TVCActuator_Sim::enqueue_frame(const CanFrame& frame) {
    if (!accepts_frame(frame)) return;

    if (running_ && rx_channel_) {
        rx_channel_->push(frame);
    } else {
        process_frame(frame);
    }
}

void TVCActuator_Sim::start(int priority) {
    if (running_) return;
    running_ = true;
    rx_channel_ = std::make_unique<boost::fibers::buffered_channel<CanFrame>>(queue_capacity_);

    fiber_ = launch_fiber_with_priority(priority, [this]() {
        this->fiber_loop();
    });
}

void TVCActuator_Sim::stop() {
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

void TVCActuator_Sim::join() {
    if (fiber_.joinable()) {
        fiber_.join();
    }
}

void TVCActuator_Sim::fiber_loop() {
    while (running_) {
        if (rx_channel_) {
            CanFrame frame;
            while (rx_channel_->try_pop(frame) == boost::fibers::channel_op_status::success) {
                process_frame(frame);
            }
        }

        float dt_sec = static_cast<float>(timestep_us_) / 1000000.0f;
        update_physics(dt_sec);

        VirtualClock::instance().sleep_for(timestep_us_);
    }
}

void TVCActuator_Sim::process_frame(const CanFrame& frame) {
    if (frame.len < 1) return;

    uint8_t cmd = frame.data[0];

    // Command 0x20: Set actuator length
    // byte 1..2: 16-bit integer length in 0.1 mm units
    if (cmd == 0x20 && frame.len >= 3) {
        uint16_t fixed_len = static_cast<uint16_t>((frame.data[1] << 8) | frame.data[2]);
        set_target_length_mm(static_cast<float>(fixed_len) / 10.0f);
        command_count_++;
    }
    // Command 0x21: Query actuator status
    else if (cmd == 0x21) {
        command_count_++;
        transmit_telemetry();
    }
}

void TVCActuator_Sim::update_physics(float dt_sec) {
    if (dt_sec <= 0.0f) return;

    float max_delta = max_speed_mm_s_ * dt_sec;
    float delta = target_length_mm_ - current_length_mm_;

    if (std::abs(delta) <= max_delta) {
        current_length_mm_ = target_length_mm_;
    } else if (delta > 0.0f) {
        current_length_mm_ += max_delta;
    } else {
        current_length_mm_ -= max_delta;
    }
}

void TVCActuator_Sim::transmit_telemetry() {
    auto b = bus_.lock();
    if (!b) return;

    CanFrame tx_frame;
    tx_frame.id = can_id_;
    tx_frame.extended = false;
    tx_frame.rtr = false;
    tx_frame.len = 4;

    uint16_t fixed_len = static_cast<uint16_t>(current_length_mm_ * 10.0f);
    tx_frame.data[0] = 0x22; // Actuator status reply
    tx_frame.data[1] = static_cast<uint8_t>((fixed_len >> 8) & 0xFF);
    tx_frame.data[2] = static_cast<uint8_t>(fixed_len & 0xFF);
    tx_frame.data[3] = (current_length_mm_ == target_length_mm_) ? 0x01 : 0x00; // 1 = at setpoint

    b->broadcast(tx_frame, this);
}

} // namespace toad::sim

