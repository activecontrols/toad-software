#include "VirtualClock.h"

VirtualClock& VirtualClock::instance() {
    static VirtualClock inst;
    return inst;
}

uint64_t VirtualClock::now_us() const {
    std::unique_lock<boost::fibers::mutex> lk(clock_mtx_);
    return current_micros_;
}

uint32_t VirtualClock::now_ms() const {
    return static_cast<uint32_t>(now_us() / 1000ULL);
}

void VirtualClock::sleep_for(uint64_t delta_us) {
    uint64_t target = now_us() + delta_us;
    sleep_until(target);
}

void VirtualClock::sleep_until(uint64_t target_us) {
    auto cv = std::make_shared<boost::fibers::condition_variable>();
    auto mtx = std::make_shared<boost::fibers::mutex>();
    auto expired = std::make_shared<bool>(false);

    {
        std::unique_lock<boost::fibers::mutex> lk(clock_mtx_);
        if (target_us <= current_micros_) {
            lk.unlock();
            boost::this_fiber::yield();
            return;
        }

        TimerEntry entry;
        entry.deadline_us = target_us;
        entry.sequence_id = ++sequence_counter_;
        entry.cv = cv;
        entry.mtx = mtx;
        entry.expired = expired;
        timers_.push(entry);
    }

    std::unique_lock<boost::fibers::mutex> lk(*mtx);
    cv->wait(lk, [&]{ return *expired; });
}

void VirtualClock::reset(uint64_t initial_us) {
    std::unique_lock<boost::fibers::mutex> lk(clock_mtx_);
    current_micros_ = initial_us;
    sequence_counter_ = 0;
    while (!timers_.empty()) {
        timers_.pop();
    }
}

bool VirtualClock::has_pending_timers() const {
    std::unique_lock<boost::fibers::mutex> lk(clock_mtx_);
    return !timers_.empty();
}

uint64_t VirtualClock::next_deadline_us() const {
    std::unique_lock<boost::fibers::mutex> lk(clock_mtx_);
    if (timers_.empty()) {
        return current_micros_;
    }
    return timers_.top().deadline_us;
}

bool VirtualClock::step_to_next_event() {
    struct WakeEntry {
        std::shared_ptr<boost::fibers::condition_variable> cv;
        std::shared_ptr<boost::fibers::mutex> mtx;
        std::shared_ptr<bool> expired;
    };
    std::vector<WakeEntry> wake_list;

    {
        std::unique_lock<boost::fibers::mutex> lk(clock_mtx_);
        if (timers_.empty()) {
            return false;
        }

        uint64_t next_us = timers_.top().deadline_us;
        current_micros_ = next_us;

        while (!timers_.empty() && timers_.top().deadline_us <= current_micros_) {
            auto top = timers_.top();
            timers_.pop();
            wake_list.push_back({top.cv, top.mtx, top.expired});
        }
    }

    for (auto& item : wake_list) {
        {
            std::unique_lock<boost::fibers::mutex> lk(*item.mtx);
            *item.expired = true;
        }
        item.cv->notify_all();
    }

    boost::this_fiber::yield();
    return true;
}

void VirtualClock::advance_time_to(uint64_t target_us) {
    while (now_us() < target_us) {
        boost::this_fiber::yield();

        std::unique_lock<boost::fibers::mutex> lk(clock_mtx_);
        if (timers_.empty()) {
            current_micros_ = target_us;
            break;
        }

        uint64_t next_us = timers_.top().deadline_us;
        if (next_us > target_us) {
            current_micros_ = target_us;
            break;
        }

        lk.unlock();
        step_to_next_event();
    }
    boost::this_fiber::yield();
}

void VirtualClock::run_until(uint64_t max_us) {
    advance_time_to(max_us);
}

