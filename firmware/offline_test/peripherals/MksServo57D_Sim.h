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
 * @brief Behavioral simulation model for the Makerbase MKS SERVO42D / 57D closed-loop stepper motor.
 *
 * Implements:
 * - CAN 2.0 communication protocol with hardware checksum validation.
 * - Command decoding: Speed command (0xF6), position control, status queries.
 * - Continuous physical dynamics simulation: velocity ramping with acceleration and angle integration.
 * - Independent background fiber running at PRIO_ACTUATOR_PHYSICS (0).
 */
class MksServo57D_Sim : public ICANDevice, public std::enable_shared_from_this<MksServo57D_Sim> {
public:
    explicit MksServo57D_Sim(std::string name, uint32_t can_id = 0x003);
    ~MksServo57D_Sim() override;

    const std::string& name() const override { return name_; }
    uint32_t can_id() const override { return can_id_; }

    void enqueue_frame(const CanFrame& frame) override;

    // Concurrency Lifecycle
    void start(int priority = PRIO_ACTUATOR_PHYSICS) override;
    void stop() override;
    void join() override;
    bool is_running() const override { return running_; }

    // Physical state inspection & manipulation
    int16_t target_speed_rpm() const { return target_speed_rpm_; }
    float current_speed_rpm() const { return current_speed_rpm_; }
    float current_angle_deg() const { return current_angle_deg_; }
    uint8_t acceleration() const { return acceleration_; }

    void set_current_angle_deg(float angle_deg) { current_angle_deg_ = angle_deg; }
    void set_integration_timestep_us(uint64_t us) { timestep_us_ = us; }

    uint32_t command_count() const { return command_count_; }
    uint32_t crc_error_count() const { return crc_error_count_; }

    // Transmit telemetry frame back over the CAN bus
    void transmit_telemetry();

    // Checksum calculator matching MKS Servo specification
    static uint8_t calculate_checksum(uint16_t can_id, const uint8_t* data, size_t len_without_crc);

private:
    void fiber_loop();
    void process_frame(const CanFrame& frame);
    void update_physics(float dt_sec);

    std::string name_;
    uint32_t can_id_{0x003};

    bool running_{false};
    uint64_t timestep_us_{1000}; // 1 ms physics integration timestep
    size_t queue_capacity_{32};

    int16_t target_speed_rpm_{0};
    float current_speed_rpm_{0.0f};
    float current_angle_deg_{0.0f};
    uint8_t acceleration_{32};

    uint32_t command_count_{0};
    uint32_t crc_error_count_{0};

    std::unique_ptr<boost::fibers::buffered_channel<CanFrame>> rx_channel_{nullptr};
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim

