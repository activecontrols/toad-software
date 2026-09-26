#include "SimulatedGPIO.h"
#include <stdio.h>
#include <boost/fiber/all.hpp>

// for pin_size_t:
#include "api/Common.h"

namespace toad::sim {

using guard = std::lock_guard<boost::fibers::mutex>;

SimulatedGPIO& SimulatedGPIO::instance() {
    static SimulatedGPIO instance;
    return instance;
}

void SimulatedGPIO::set_pin_mode(uint32_t pin, arduino::PinMode mode) {
    guard lock(mtx_);
    pins_[pin].mode = mode;
}

arduino::PinMode SimulatedGPIO::get_pin_mode(uint32_t pin) const {
    guard lock(mtx_);
    auto it = pins_.find(pin);
    if (it != pins_.end()) {
        return it->second.mode;
    }
    return arduino::INPUT;
}

void SimulatedGPIO::write_pin(uint32_t pin, arduino::PinStatus status) {
    arduino::PinStatus old_status = arduino::LOW;
    std::vector<PinListener> pin_listeners;

    {
        guard lock(mtx_);
        auto& p = pins_[pin];
        old_status = p.digital_val;
        p.digital_val = status;

        auto it = listeners_.find(pin);
        if (it != listeners_.end()) {
            pin_listeners = it->second;
        }
    }

    if (old_status != status) {
        for (const auto& listener : pin_listeners) {
            listener(pin, status, old_status);
        }

        printf("Pin %d state: %d\n", pin, status == arduino::PinStatus::HIGH ? 1 : 0);
    }
}

arduino::PinStatus SimulatedGPIO::read_pin(uint32_t pin) const {
    guard lock(mtx_);
    auto it = pins_.find(pin);
    if (it != pins_.end()) {
        return it->second.digital_val;
    }
    return arduino::LOW;
}

void SimulatedGPIO::write_analog(uint32_t pin, int val) {
    guard lock(mtx_);
    pins_[pin].analog_val = val;
}

int SimulatedGPIO::read_analog(uint32_t pin) const {
    guard lock(mtx_);
    auto it = pins_.find(pin);
    if (it != pins_.end()) {
        return it->second.analog_val;
    }
    return 0;
}

void SimulatedGPIO::add_listener(uint32_t pin, PinListener listener) {
    guard lock(mtx_);
    listeners_[pin].push_back(std::move(listener));
}

void SimulatedGPIO::remove_listener(uint32_t pin) {
    guard lock(mtx_);
    listeners_.erase(pin);
}

void SimulatedGPIO::reset() {
    guard lock(mtx_);
    pins_.clear();
    listeners_.clear();
}

} // namespace toad::sim

