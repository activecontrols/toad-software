#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <vector>

#include <boost/fiber/fiber.hpp>
#include <boost/fiber/buffered_channel.hpp>

#include "ICANDevice.h"
#include "bus/CanFrame.h"
#include "core/FiberScheduler.h"

#include "core/VirtualClock.h"

namespace toad::sim {

/**
 * @brief Behavioral simulation model for Thrust Vector Control (TVC) linear actuator.
 *
 * Implements:
 * - CAN position commands for stroke length (mm) and gimbal articulation.
 * - Dynamic linear velocity and extension/retraction physics.
 * - Independent background fiber running at PRIO_ACTUATOR_PHYSICS (0).
 */
class TVCActuator_Sim : public ICANDevice, public std::enable_shared_from_this<TVCActuator_Sim> {
public:
    explicit TVCActuator_Sim(std::string name, uint32_t can_id = 0x001, float initial_length_mm = 50.0f);
    ~TVCActuator_Sim() override;

    const std::string& name() const override { return name_; }
    uint32_t can_id() const override { return can_id_; }

    void enqueue_frame(const CanFrame& frame) override;

    // Concurrency Lifecycle
    void start(int priority = PRIO_ACTUATOR_PHYSICS) override;
    void stop() override;
    void join() override;
    bool is_running() const override { return running_; }

    // Physical state inspection & manipulation
    float target_length_mm() const { return target_length_mm_; }
    float current_length_mm() const { return current_length_mm_; }
    float max_speed_mm_s() const { return max_speed_mm_s_; }

    void set_target_length_mm(float target_mm);
    void set_current_length_mm(float length_mm) { current_length_mm_ = length_mm; }
    void set_max_speed_mm_s(float speed) { max_speed_mm_s_ = speed; }
    void set_stroke_limits(float min_mm, float max_mm) { min_length_mm_ = min_mm; max_length_mm_ = max_mm; }

    uint32_t command_count() const { return command_count_; }

    void transmit_telemetry();

private:
    void fiber_loop();
    void process_frame(const CanFrame& frame);
    void update_physics(float dt_sec);

    std::string name_;
    uint32_t can_id_{0x001};

    bool running_{false};
    uint64_t timestep_us_{1000}; // 1 ms physics integration timestep
    size_t queue_capacity_{32};

    float current_length_mm_{50.0f};
    float target_length_mm_{50.0f};
    float min_length_mm_{0.0f};
    float max_length_mm_{100.0f};
    float max_speed_mm_s_{25.0f}; // 25 mm/s linear velocity

    uint32_t command_count_{0};

    std::unique_ptr<boost::fibers::buffered_channel<CanFrame>> rx_channel_{nullptr};
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim

