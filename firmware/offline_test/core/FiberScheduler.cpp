#include "FiberScheduler.h"
#include <algorithm>

namespace toad::sim {

void PriorityScheduler::awakened(boost::fibers::context* ctx, PriorityProps& props) noexcept {
    int p = props.get_priority();
    // Maintain descending order: highest priority first
    auto it = std::find_if(ready_queue_.begin(), ready_queue_.end(),
                           [p](const ReadyEntry& entry) { return entry.priority < p; });
    ready_queue_.insert(it, {ctx, p});
}

boost::fibers::context* PriorityScheduler::pick_next() noexcept {
    if (ready_queue_.empty()) {
        return nullptr;
    }
    boost::fibers::context* ctx = ready_queue_.front().ctx;
    ready_queue_.erase(ready_queue_.begin());
    return ctx;
}

bool PriorityScheduler::has_ready_fibers() const noexcept {
    return !ready_queue_.empty();
}

void PriorityScheduler::property_change(boost::fibers::context* ctx, PriorityProps& props) noexcept {
    auto it = std::find_if(ready_queue_.begin(), ready_queue_.end(),
                           [ctx](const ReadyEntry& entry) { return entry.ctx == ctx; });
    if (it != ready_queue_.end()) {
        ready_queue_.erase(it);
        awakened(ctx, props);
    }
}

void PriorityScheduler::suspend_until(std::chrono::steady_clock::time_point const& suspend_time) noexcept {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait_until(lk, suspend_time, [this]() { return flag_; });
    flag_ = false;
}

void PriorityScheduler::notify() noexcept {
    {
        std::unique_lock<std::mutex> lk(mtx_);
        flag_ = true;
    }
    cv_.notify_all();
}

void install_fiber_scheduler() {
    boost::fibers::use_scheduling_algorithm<PriorityScheduler>();
}

} // namespace toad::sim

