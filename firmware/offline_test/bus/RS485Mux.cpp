#include "RS485Mux.h"
#include "core/SimulatedGPIO.h"

namespace toad::sim {

RS485Mux::RS485Mux(unsigned long baud, std::string bus_name, uint32_t de_pin)
    : UartBus(baud, std::move(bus_name)), de_pin_(de_pin) {}

RS485Mux::~RS485Mux() {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    for (const auto& kv : devices_) {
        SimulatedGPIO::instance().remove_listener(kv.first);
    }
    devices_.clear();
}

void RS485Mux::register_device(uint32_t sel_pin, std::shared_ptr<IRS485Device> device) {
    if (!device) return;

    {
        std::lock_guard<std::mutex> lock(mux_mtx_);
        devices_[sel_pin] = device;
    }

    device->set_bus_transmitter([this, dev_ptr = device.get()](const uint8_t* buf, size_t sz) {
        this->receive_from_device(dev_ptr, buf, sz);
    });

    SimulatedGPIO::instance().add_listener(sel_pin, [this](uint32_t pin, arduino::PinStatus new_status, arduino::PinStatus old_status) {
        this->on_sel_pin_changed(pin, new_status, old_status);
    });

    update_mux_state();
}

void RS485Mux::unregister_device(uint32_t sel_pin) {
    SimulatedGPIO::instance().remove_listener(sel_pin);
    {
        std::lock_guard<std::mutex> lock(mux_mtx_);
        devices_.erase(sel_pin);
    }
    update_mux_state();
}

std::shared_ptr<IRS485Device> RS485Mux::get_device(uint32_t sel_pin) const {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    auto it = devices_.find(sel_pin);
    if (it != devices_.end()) {
        return it->second;
    }
    return nullptr;
}

uint32_t RS485Mux::active_sel_pin() const {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    return active_sel_pin_;
}

std::shared_ptr<IRS485Device> RS485Mux::active_device() const {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    return active_device_;
}

bool RS485Mux::has_bus_contention() const {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    return bus_contention_;
}

size_t RS485Mux::contention_count() const {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    return contention_count_;
}

void RS485Mux::reset_contention() {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    bus_contention_ = false;
    contention_count_ = 0;
}

void RS485Mux::on_sel_pin_changed(uint32_t pin, arduino::PinStatus new_status, arduino::PinStatus old_status) {
    (void)pin;
    (void)new_status;
    (void)old_status;
    update_mux_state();
}

void RS485Mux::update_mux_state() {
    std::lock_guard<std::mutex> lock(mux_mtx_);
    std::vector<uint32_t> high_pins;

    for (const auto& kv : devices_) {
        if (SimulatedGPIO::instance().read_pin(kv.first) == arduino::HIGH) {
            high_pins.push_back(kv.first);
        }
    }

    if (high_pins.size() > 1) {
        bus_contention_ = true;
        contention_count_++;
        active_sel_pin_ = NC;
        active_device_ = nullptr;
    } else if (high_pins.size() == 1) {
        bus_contention_ = false;
        active_sel_pin_ = high_pins[0];
        active_device_ = devices_[active_sel_pin_];
    } else {
        bus_contention_ = false;
        active_sel_pin_ = NC;
        active_device_ = nullptr;
    }
}

void RS485Mux::write_from_firmware(uint8_t byte) {
    write_from_firmware(&byte, 1);
}

void RS485Mux::write_from_firmware(const uint8_t* buffer, size_t size) {
    if (!buffer || size == 0) return;

    // Record transaction on the bus and apply virtual clock baud delay
    UartBus::write_from_firmware(buffer, size);

    std::shared_ptr<IRS485Device> target_dev;
    bool in_contention = false;

    {
        std::lock_guard<std::mutex> lock(mux_mtx_);
        in_contention = bus_contention_;
        target_dev = active_device_;
    }

    // Only route to device if bus is clear (no contention) and a device is selected
    if (!in_contention && target_dev) {
        target_dev->on_receive_bytes(buffer, size);
    }
}

void RS485Mux::receive_from_device(IRS485Device* device, const uint8_t* buffer, size_t size) {
    if (!device || !buffer || size == 0) return;

    bool allow_transmit = false;
    {
        std::lock_guard<std::mutex> lock(mux_mtx_);
        // Device can only transmit if it is currently selected and there is no contention
        if (!bus_contention_ && active_device_.get() == device) {
            allow_transmit = true;
        }
    }

    if (allow_transmit) {
        UartBus::write_to_firmware(buffer, size);
    }
}

} // namespace toad::sim

