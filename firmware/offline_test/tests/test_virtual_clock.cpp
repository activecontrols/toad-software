#include <iostream>
#include <chrono>
#include <cassert>
#include <vector>
#include <string>

#include "VirtualClock.h"
#include "FiberScheduler.h"
#include "Arduino.h"

// Enforce 32-bit target architecture matching embedded microcontroller
static_assert(sizeof(void*) == 4, "Simulation binary must be compiled for 32-bit target (sizeof(void*) == 4)");
static_assert(sizeof(size_t) == 4, "Simulation binary size_t must be 32-bit");

using namespace toad::sim;

void test_clock_jump() {
    std::cout << "[Test 1] Testing discrete clock jump..." << std::endl;
    VirtualClock::instance().reset(0);
    assert(micros() == 0);
    assert(millis() == 0);

    bool fiber_executed = false;
    uint64_t recorded_wake_time = 0;

    auto start_wall = std::chrono::steady_clock::now();

    boost::fibers::fiber f([&]() {
        delay(3000); // 3-second virtual delay (3,000,000 us)
        fiber_executed = true;
        recorded_wake_time = micros();
    });

    // Step clock until completion
    VirtualClock::instance().run_until(3000000);
    f.join();

    auto elapsed_wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_wall
    ).count();

    assert(fiber_executed);
    assert(recorded_wake_time == 3000000);
    assert(millis() == 3000);
    std::cout << "  -> 3.0s virtual time advanced in " << elapsed_wall_ms << " ms wall-clock time!" << std::endl;
    assert(elapsed_wall_ms < 500); // Must be near-instantaneous, not waiting 3 wall seconds
    std::cout << "  -> PASSED" << std::endl;
}

void test_priority_scheduling() {
    std::cout << "[Test 2] Testing priority ordering on simultaneous wakeups..." << std::endl;
    VirtualClock::instance().reset(0);

    std::vector<std::string> execution_order;

    // Both fibers sleep for the exact same duration: 5000 us
    auto f_low = launch_fiber_with_priority(PRIO_ACTUATOR_PHYSICS, [&]() {
        delayMicroseconds(5000);
        execution_order.push_back("LOW_PRIO_ACTUATOR");
    });

    auto f_high = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        delayMicroseconds(5000);
        execution_order.push_back("HIGH_PRIO_FIRMWARE");
    });

    VirtualClock::instance().run_until(5000);
    f_low.join();
    f_high.join();

    assert(execution_order.size() == 2);
    std::cout << "  First executed: " << execution_order[0] << std::endl;
    std::cout << "  Second executed: " << execution_order[1] << std::endl;
    assert(execution_order[0] == "HIGH_PRIO_FIRMWARE");
    assert(execution_order[1] == "LOW_PRIO_ACTUATOR");
    std::cout << "  -> PASSED" << std::endl;
}

void test_interleaved_delays() {
    std::cout << "[Test 3] Testing interleaved virtual delays..." << std::endl;
    VirtualClock::instance().reset(0);

    std::vector<std::pair<uint64_t, std::string>> timeline;

    auto f1 = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        delayMicroseconds(100);
        timeline.push_back({micros(), "F1 @ 100"});
        delayMicroseconds(200);
        timeline.push_back({micros(), "F1 @ 300"});
    });

    auto f2 = launch_fiber_with_priority(PRIO_SENSORS, [&]() {
        delayMicroseconds(150);
        timeline.push_back({micros(), "F2 @ 150"});
        delayMicroseconds(200);
        timeline.push_back({micros(), "F2 @ 350"});
    });

    VirtualClock::instance().run_until(400);
    f1.join();
    f2.join();

    assert(timeline.size() == 4);
    for (const auto& event : timeline) {
        std::cout << "  [" << event.first << " us] " << event.second << std::endl;
    }

    assert(timeline[0].first == 100);
    assert(timeline[1].first == 150);
    assert(timeline[2].first == 300);
    assert(timeline[3].first == 350);
    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "=== Running VirtualClock & FiberScheduler Verification Tests ===" << std::endl;
    install_fiber_scheduler();

    test_clock_jump();
    test_priority_scheduling();
    test_interleaved_delays();

    std::cout << "=== All Milestone 1 Tests Passed Successfully! ===" << std::endl;
    return 0;
}

