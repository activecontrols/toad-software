#pragma once

#include <cstdint>
#include <memory>
#include <queue>
#include <vector>
#include <boost/fiber/all.hpp>

class VirtualClock {
public:
    static VirtualClock& instance();

    // Current time queries
    uint64_t now_us() const;
    uint32_t now_ms() const;

    // Fiber sleep methods (suspends calling fiber)
    void sleep_for(uint64_t delta_us);
    void sleep_until(uint64_t target_us);

    // Simulation stepping & inspection
    void reset(uint64_t initial_us = 0);
    bool has_pending_timers() const;
    uint64_t next_deadline_us() const;
    bool step_to_next_event();

    // Advance virtual time up to target_us, waking and running fibers at each deadline
    void advance_time_to(uint64_t target_us);

    // Convenience stepper: runs until all timers/fibers complete or max_us reached
    void run_until(uint64_t max_us);

private:
    VirtualClock() = default;
    ~VirtualClock() = default;
    VirtualClock(const VirtualClock&) = delete;
    VirtualClock& operator=(const VirtualClock&) = delete;

    struct TimerEntry {
        uint64_t deadline_us;
        uint64_t sequence_id;
        std::shared_ptr<boost::fibers::condition_variable> cv;
        std::shared_ptr<boost::fibers::mutex> mtx;
        std::shared_ptr<bool> expired;

        bool operator>(const TimerEntry& other) const {
            if (deadline_us != other.deadline_us) {
                return deadline_us > other.deadline_us;
            }
            return sequence_id > other.sequence_id;
        }
    };

    uint64_t current_micros_{0};
    uint64_t sequence_counter_{0};
    mutable boost::fibers::mutex clock_mtx_;
    std::priority_queue<TimerEntry, std::vector<TimerEntry>, std::greater<TimerEntry>> timers_;
};

