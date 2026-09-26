#pragma once

#include <cstdint>
#include <map>
#include <functional>
#include <mutex>
#include <vector>
#include "hal_mock/pins_arduino.h"
#include "api/Common.h"
#include <boost/fiber/all.hpp>

namespace toad::sim {

using PinListener = std::function<void(uint32_t pin, arduino::PinStatus new_status, arduino::PinStatus old_status)>;

struct PinState {
    arduino::PinMode mode{arduino::INPUT};
    arduino::PinStatus digital_val{arduino::LOW};
    int analog_val{0};
};

class SimulatedGPIO {
public:
    static SimulatedGPIO& instance();

    // Configuration & State Queries
    void set_pin_mode(uint32_t pin, arduino::PinMode mode);
    arduino::PinMode get_pin_mode(uint32_t pin) const;

    void write_pin(uint32_t pin, arduino::PinStatus status);
    arduino::PinStatus read_pin(uint32_t pin) const;

    void write_analog(uint32_t pin, int val);
    int read_analog(uint32_t pin) const;

    // Pin-Change Listeners (e.g. for RS-485 SEL lines or interrupts)
    void add_listener(uint32_t pin, PinListener listener);
    void remove_listener(uint32_t pin);

    // Reset all pins and listeners
    void reset();

private:
    SimulatedGPIO() = default;
    ~SimulatedGPIO() = default;
    SimulatedGPIO(const SimulatedGPIO&) = delete;
    SimulatedGPIO& operator=(const SimulatedGPIO&) = delete;

    mutable boost::fibers::mutex mtx_; // use fibers mutex; all gpio action must be performed from fiber context
    std::map<uint32_t, PinState> pins_;
    std::map<uint32_t, std::vector<PinListener>> listeners_;
};

} // namespace toad::sim

