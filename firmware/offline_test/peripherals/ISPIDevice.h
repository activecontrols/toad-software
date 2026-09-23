#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include <vector>

#include <boost/fiber/fiber.hpp>
#include <boost/fiber/buffered_channel.hpp>
#include "core/FiberScheduler.h"
#include "core/VirtualClock.h"

namespace toad::sim {

/**
 * @brief Base abstract interface for any hardware device connected to an SPI bus.
 *
 * Supports:
 * 1. Synchronous full-duplex byte/buffer clocking (transfer_byte, transfer_buffer).
 * 2. Independent background fiber execution via start(priority), stop(), join().
 */
class ISPIDevice {
public:
    virtual ~ISPIDevice() = default;

    // Device identification for debugging and transaction logs
    virtual const std::string& name() const = 0;

    // Optional fiber lifecycle hooks
    virtual void start(int priority = PRIO_SENSORS) { (void)priority; }
    virtual void stop() {}
    virtual void join() {}
    virtual bool is_running() const { return false; }

    // Chip select edge notifications (called when CS is asserted active or deasserted inactive)
    virtual void on_cs_asserted() {}
    virtual void on_cs_deasserted() {}

    // Full-duplex single byte transfer (MOSI in -> MISO out)
    virtual uint8_t transfer_byte(uint8_t mosi_byte) = 0;

    // Full-duplex buffer transfer (default implementation sequentially clocks transfer_byte)
    virtual void transfer_buffer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            uint8_t mosi = tx_buf ? tx_buf[i] : 0xFF;
            uint8_t miso = transfer_byte(mosi);
            if (rx_buf) {
                rx_buf[i] = miso;
            }
        }
    }
};

/**
 * @brief Lambda-based pluggable SPI device with optional background fiber concurrency.
 *
 * Synchronous usage:
 *   auto my_sensor = std::make_shared<FunctionalSPIDevice>("MyADC",
 *       [](uint8_t mosi, FunctionalSPIDevice& dev) -> uint8_t {
 *           return 0x42;
 *       });
 *
 * Concurrent background worker usage:
 *   my_sensor->set_worker([](FunctionalSPIDevice& dev) {
 *       while (dev.is_running()) {
 *           VirtualClock::instance().sleep_for(1000);
 *           // update internal values
 *       }
 *   });
 *   my_sensor->start(PRIO_SENSORS);
 */
class FunctionalSPIDevice : public ISPIDevice {
public:
    using ByteHandler = std::function<uint8_t(uint8_t mosi, FunctionalSPIDevice& dev)>;
    using CsHandler = std::function<void(FunctionalSPIDevice& dev)>;
    using WorkerFn = std::function<void(FunctionalSPIDevice& dev)>;
    using FrameHandler = std::function<void(const std::vector<uint8_t>& frame, FunctionalSPIDevice& dev)>;

    explicit FunctionalSPIDevice(std::string name, ByteHandler byte_handler)
        : name_(std::move(name)), byte_handler_(std::move(byte_handler)) {}

    FunctionalSPIDevice(std::string name,
                        ByteHandler byte_handler,
                        CsHandler on_assert,
                        CsHandler on_deassert)
        : name_(std::move(name)),
          byte_handler_(std::move(byte_handler)),
          on_assert_(std::move(on_assert)),
          on_deassert_(std::move(on_deassert)) {}

    ~FunctionalSPIDevice() override {
        stop();
    }

    const std::string& name() const override { return name_; }

    void set_worker(WorkerFn worker) {
        worker_ = std::move(worker);
    }

    void set_frame_handler(FrameHandler frame_handler, size_t queue_capacity = 16) {
        frame_handler_ = std::move(frame_handler);
        queue_capacity_ = queue_capacity;
    }

    void start(int priority = PRIO_SENSORS) override {
        if (running_) return;
        running_ = true;

        if (frame_handler_) {
            rx_frame_channel_ = std::make_unique<boost::fibers::buffered_channel<std::vector<uint8_t>>>(queue_capacity_);
        }

        fiber_ = launch_fiber_with_priority(priority, [this]() {
            if (worker_) {
                worker_(*this);
            } else if (frame_handler_ && rx_frame_channel_) {
                while (running_) {
                    std::vector<uint8_t> frame;
                    auto status = rx_frame_channel_->pop(frame);
                    if (status == boost::fibers::channel_op_status::closed || !running_) {
                        break;
                    }
                    if (!frame.empty()) {
                        frame_handler_(frame, *this);
                    }
                }
            }
        });
    }

    void stop() override {
        if (!running_) return;
        running_ = false;
        VirtualClock::instance().wake_all();
        if (rx_frame_channel_) {
            rx_frame_channel_->close();
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

    void on_cs_asserted() override {
        current_frame_bytes_.clear();
        if (on_assert_) on_assert_(*this);
    }

    void on_cs_deasserted() override {
        if (on_deassert_) on_deassert_(*this);

        if (running_ && rx_frame_channel_ && !current_frame_bytes_.empty()) {
            rx_frame_channel_->push(std::move(current_frame_bytes_));
            current_frame_bytes_.clear();
        }
    }

    uint8_t transfer_byte(uint8_t mosi_byte) override {
        if (running_ && rx_frame_channel_) {
            current_frame_bytes_.push_back(mosi_byte);
        }
        if (byte_handler_) {
            return byte_handler_(mosi_byte, *this);
        }
        return 0xFF; // Floating MISO line default
    }

private:
    std::string name_;
    ByteHandler byte_handler_;
    CsHandler on_assert_{nullptr};
    CsHandler on_deassert_{nullptr};
    WorkerFn worker_{nullptr};
    FrameHandler frame_handler_{nullptr};

    size_t queue_capacity_{16};
    bool running_{false};
    std::unique_ptr<boost::fibers::buffered_channel<std::vector<uint8_t>>> rx_frame_channel_{nullptr};
    std::vector<uint8_t> current_frame_bytes_;
    boost::fibers::fiber fiber_;
};

} // namespace toad::sim
