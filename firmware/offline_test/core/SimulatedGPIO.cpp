#include "SimulatedGPIO.h"
#include <stdio.h>

namespace toad::sim {

SimulatedGPIO& SimulatedGPIO::instance() {
    static SimulatedGPIO instance;
    return instance;
}

void SimulatedGPIO::set_pin_mode(uint32_t pin, arduino::PinMode mode) {
    std::lock_guard<std::mutex> lock(mtx_);
    pins_[pin].mode = mode;
}

arduino::PinMode SimulatedGPIO::get_pin_mode(uint32_t pin) const {
    std::lock_guard<std::mutex> lock(mtx_);
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
        std::lock_guard<std::mutex> lock(mtx_);
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
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = pins_.find(pin);
    if (it != pins_.end()) {
        return it->second.digital_val;
    }
    return arduino::LOW;
}

void SimulatedGPIO::write_analog(uint32_t pin, int val) {
    std::lock_guard<std::mutex> lock(mtx_);
    pins_[pin].analog_val = val;
}

int SimulatedGPIO::read_analog(uint32_t pin) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = pins_.find(pin);
    if (it != pins_.end()) {
        return it->second.analog_val;
    }
    return 0;
}

void SimulatedGPIO::add_listener(uint32_t pin, PinListener listener) {
    std::lock_guard<std::mutex> lock(mtx_);
    listeners_[pin].push_back(std::move(listener));
}

void SimulatedGPIO::remove_listener(uint32_t pin) {
    std::lock_guard<std::mutex> lock(mtx_);
    listeners_.erase(pin);
}

void SimulatedGPIO::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    pins_.clear();
    listeners_.clear();
}

} // namespace toad::sim

