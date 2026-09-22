#pragma once

#include <boost/fiber/all.hpp>
#include <boost/fiber/algo/algorithm.hpp>
#include <vector>
#include <mutex>
#include <condition_variable>

namespace toad::sim {

// Standard task priority levels for Toad SITL
constexpr int PRIO_FIRMWARE = 10;
constexpr int PRIO_SENSORS = 5;
constexpr int PRIO_ACTUATOR_PHYSICS = 0;

class PriorityProps : public boost::fibers::fiber_properties {
public:
    PriorityProps(boost::fibers::context* ctx)
        : boost::fibers::fiber_properties(ctx), priority_(0) {}

    int get_priority() const noexcept { return priority_; }

    void set_priority(int p) {
        if (p == priority_) return;
        priority_ = p;
        notify();
    }

private:
    int priority_;
};

class PriorityScheduler : public boost::fibers::algo::algorithm_with_properties<PriorityProps> {
public:
    PriorityScheduler() = default;
    ~PriorityScheduler() override = default;

    void awakened(boost::fibers::context* ctx, PriorityProps& props) noexcept override;
    boost::fibers::context* pick_next() noexcept override;
    bool has_ready_fibers() const noexcept override;
    void property_change(boost::fibers::context* ctx, PriorityProps& props) noexcept override;
    void suspend_until(std::chrono::steady_clock::time_point const& suspend_time) noexcept override;
    void notify() noexcept override;

private:
    struct ReadyEntry {
        boost::fibers::context* ctx;
        int priority;
    };

    std::vector<ReadyEntry> ready_queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool flag_{false};
};

// Installs PriorityScheduler on the calling thread
void install_fiber_scheduler();

// Helper to launch a fiber with an assigned priority
template <typename Fn, typename... Args>
boost::fibers::fiber launch_fiber_with_priority(int priority, Fn&& fn, Args&&... args) {
    boost::fibers::fiber f(std::forward<Fn>(fn), std::forward<Args>(args)...);
    f.properties<PriorityProps>().set_priority(priority);
    return f;
}

} // namespace toad::sim

