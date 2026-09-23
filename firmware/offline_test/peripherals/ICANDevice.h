#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <functional>

#include <boost/fiber/fiber.hpp>
#include <boost/fiber/buffered_channel.hpp>

#include "bus/CanFrame.h"
#include "core/FiberScheduler.h"
#include "core/VirtualClock.h"


namespace toad::sim {

class CANBus;

/**
 * @brief Base abstract interface for any hardware node connected to a CAN bus.
 *
 * Implements:
 * 1. Acceptance filtering via can_id() and can_mask().
 * 2. Asynchronous queuing: enqueue_frame() pushes frames into private device queue.
 * 3. Cooperative concurrency: start(priority), stop(), join(), is_running().
 */
class ICANDevice {
public:
    virtual ~ICANDevice() = default;

    virtual const std::string& name() const = 0;
    virtual uint32_t can_id() const = 0;
    virtual uint32_t can_mask() const { return 0x7FF; } // Standard 11-bit mask by default

    /**
     * @brief Evaluates whether this node accepts the given CAN ID.
     */
    virtual bool accepts_frame(const CanFrame& frame) const {
        return (frame.id & can_mask()) == (can_id() & can_mask());
    }

    /**
     * @brief Called by CANBus to deliver a broadcast frame to this device.
     */
    virtual void enqueue_frame(const CanFrame& frame) = 0;

    /**
     * @brief Associates this device with its host CANBus fabric.
     */
    virtual void set_bus(std::shared_ptr<CANBus> bus) { bus_ = bus; }
    virtual std::shared_ptr<CANBus> bus() const { return bus_.lock(); }

    /**
     * @brief Fiber lifecycle methods for concurrent background execution.
     */
    virtual void start(int priority = PRIO_ACTUATOR_PHYSICS) = 0;
    virtual void stop() = 0;
    virtual void join() = 0;
    virtual bool is_running() const = 0;

protected:
    std::weak_ptr<CANBus> bus_;
};

/**
 * @brief Rapid lambda-based CAN node mock for 3-line test substitutions.
 */
class FunctionalCANDevice : public ICANDevice, public std::enable_shared_from_this<FunctionalCANDevice> {
public:
    using FrameHandler = std::function<void(const CanFrame& frame, FunctionalCANDevice& dev)>;
    using WorkerFn = std::function<void(FunctionalCANDevice& dev)>;

    FunctionalCANDevice(std::string name, uint32_t can_id, FrameHandler handler = nullptr, uint32_t can_mask = 0x7FF)
        : name_(std::move(name)), can_id_(can_id), can_mask_(can_mask), handler_(std::move(handler)) {}

    ~FunctionalCANDevice() override {
        stop();
    }

    const std::string& name() const override { return name_; }
    uint32_t can_id() const override { return can_id_; }
    uint32_t can_mask() const override { return can_mask_; }

    void set_handler(FrameHandler handler) { handler_ = std::move(handler); }
    void set_worker(WorkerFn worker) { worker_ = std::move(worker); }
    void set_queue_capacity(size_t capacity) { queue_capacity_ = capacity; }

    void enqueue_frame(const CanFrame& frame) override {
        if (!accepts_frame(frame)) return;

        if (running_ && rx_channel_) {
            rx_channel_->push(frame);
        } else if (handler_) {
            handler_(frame, *this);
        }
    }

    void transmit(const CanFrame& frame);

    void start(int priority = PRIO_ACTUATOR_PHYSICS) override {
        if (running_) return;
        running_ = true;
        rx_channel_ = std::make_unique<boost::fibers::buffered_channel<CanFrame>>(queue_capacity_);

        fiber_ = launch_fiber_with_priority(priority, [this]() {
            if (worker_) {
                worker_(*this);
            } else {
                while (running_) {
                    CanFrame frame;
                    auto status = rx_channel_->pop(frame);
                    if (status == boost::fibers::channel_op_status::closed || !running_) {
                        break;
                    }
                    if (handler_) {
                        handler_(frame, *this);
                    }
                }
            }
        });
    }

    void stop() override {
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

    void join() override {
        if (fiber_.joinable()) {
            fiber_.join();
        }
    }

    bool is_running() const override { return running_; }

private:
    std::string name_;
    uint32_t can_id_{0};
    uint32_t can_mask_{0x7FF};
    FrameHandler handler_{nullptr};
    WorkerFn worker_{nullptr};
    size_t queue_capacity_{32};
    bool running_{false};
    std::unique_ptr<boost::fibers::buffered_channel<CanFrame>> rx_channel_{nullptr};
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim

