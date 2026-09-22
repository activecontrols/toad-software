#include "SPIBus.h"
#include "core/SimulatedGPIO.h"
#include "core/VirtualClock.h"
#include <algorithm>

namespace toad::sim {

SPIBus::SPIBus(std::string bus_name, uint32_t mosi_pin, uint32_t miso_pin, uint32_t sck_pin)
    : bus_name_(std::move(bus_name)),
      mosi_pin_(mosi_pin),
      miso_pin_(miso_pin),
      sck_pin_(sck_pin) {}

SPIBus::~SPIBus() {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    for (const auto& kv : devices_) {
        SimulatedGPIO::instance().remove_listener(kv.first);
    }
    devices_.clear();
}

void SPIBus::register_device(uint32_t cs_pin, std::shared_ptr<ISPIDevice> device, bool active_low) {
    if (!device) return;

    {
        std::lock_guard<std::mutex> lock(bus_mtx_);
        devices_[cs_pin] = {device, active_low};
    }

    SimulatedGPIO::instance().add_listener(cs_pin, [this](uint32_t pin, PinStatus new_status, PinStatus old_status) {
        this->on_cs_pin_changed(pin, new_status, old_status);
    });

    update_cs_state();
}

void SPIBus::unregister_device(uint32_t cs_pin) {
    SimulatedGPIO::instance().remove_listener(cs_pin);
    {
        std::lock_guard<std::mutex> lock(bus_mtx_);
        devices_.erase(cs_pin);
    }
    update_cs_state();
}

std::shared_ptr<ISPIDevice> SPIBus::get_device(uint32_t cs_pin) const {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    auto it = devices_.find(cs_pin);
    if (it != devices_.end()) {
        return it->second.device;
    }
    return nullptr;
}

uint32_t SPIBus::active_cs_pin() const {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    return active_cs_pin_;
}

std::shared_ptr<ISPIDevice> SPIBus::active_device() const {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    return active_device_;
}

bool SPIBus::has_bus_contention() const {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    return bus_contention_;
}

size_t SPIBus::contention_count() const {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    return contention_count_;
}

void SPIBus::reset_contention() {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    bus_contention_ = false;
    contention_count_ = 0;
}

void SPIBus::on_cs_pin_changed(uint32_t pin, PinStatus new_status, PinStatus old_status) {
    (void)pin;
    (void)new_status;
    (void)old_status;
    update_cs_state();
}

void SPIBus::update_cs_state() {
    std::shared_ptr<ISPIDevice> newly_asserted = nullptr;
    std::shared_ptr<ISPIDevice> newly_deasserted = nullptr;

    {
        std::lock_guard<std::mutex> lock(bus_mtx_);
        std::vector<uint32_t> active_pins;

        for (const auto& kv : devices_) {
            uint32_t pin = kv.first;
            bool active_low = kv.second.active_low;
            PinStatus current = SimulatedGPIO::instance().read_pin(pin);
            bool is_active = active_low ? (current == ::LOW) : (current == ::HIGH);

            if (is_active) {
                active_pins.push_back(pin);
            }
        }

        if (active_pins.size() > 1) {
            bus_contention_ = true;
            contention_count_++;
            if (active_device_) {
                newly_deasserted = active_device_;
            }
            active_cs_pin_ = NC;
            active_device_ = nullptr;
        } else if (active_pins.size() == 1) {
            bus_contention_ = false;
            uint32_t pin = active_pins[0];
            if (active_cs_pin_ != pin) {
                if (active_device_) {
                    newly_deasserted = active_device_;
                }
                active_cs_pin_ = pin;
                active_device_ = devices_[pin].device;
                newly_asserted = active_device_;
            }
        } else {
            bus_contention_ = false;
            if (active_device_) {
                newly_deasserted = active_device_;
            }
            active_cs_pin_ = NC;
            active_device_ = nullptr;
        }
    }

    if (newly_deasserted) {
        newly_deasserted->on_cs_deasserted();
    }
    if (newly_asserted) {
        newly_asserted->on_cs_asserted();
    }
}

uint64_t SPIBus::calculate_byte_duration_us() const {
    if (zero_latency_) return 0;
    uint32_t freq = current_settings_.getClockFreq();
    if (freq == 0) return 0;
    // 8 bits per byte
    uint64_t duration = (8ULL * 1000000ULL) / freq;
    return duration > 0 ? duration : 1; // at least 1 us if high freq
}

void SPIBus::begin_transaction(arduino::SPISettings settings) {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    current_settings_ = settings;
    in_transaction_ = true;
    current_tx_.timestamp_us = VirtualClock::instance().now_us();
    current_tx_.cs_pin = active_cs_pin_;
    current_tx_.device_name = active_device_ ? active_device_->name() : "NONE";
    current_tx_.mosi_data.clear();
    current_tx_.miso_data.clear();
}

void SPIBus::end_transaction() {
    SpiTransaction tx_to_notify;
    bool should_notify = false;

    {
        std::lock_guard<std::mutex> lock(bus_mtx_);
        if (in_transaction_) {
            in_transaction_ = false;
            if (!current_tx_.mosi_data.empty() || !current_tx_.miso_data.empty()) {
                history_.push_back(current_tx_);
                tx_to_notify = current_tx_;
                should_notify = true;
            }
        }
    }

    if (should_notify) {
        notify_observers(tx_to_notify);
    }
}

uint8_t SPIBus::transfer(uint8_t mosi_byte) {
    uint64_t delay_us = calculate_byte_duration_us();
    if (delay_us > 0) {
        VirtualClock::instance().sleep_for(delay_us);
    }

    std::shared_ptr<ISPIDevice> target_device = nullptr;
    bool in_contention = false;

    {
        std::lock_guard<std::mutex> lock(bus_mtx_);
        in_contention = bus_contention_;
        target_device = active_device_;
    }

    uint8_t miso_byte = 0xFF; // Floating high default for SPI
    if (!in_contention && target_device) {
        miso_byte = target_device->transfer_byte(mosi_byte);
    }

    {
        std::lock_guard<std::mutex> lock(bus_mtx_);
        current_tx_.mosi_data.push_back(mosi_byte);
        current_tx_.miso_data.push_back(miso_byte);
    }

    return miso_byte;
}

uint16_t SPIBus::transfer16(uint16_t data) {
    bool msb_first = (current_settings_.getBitOrder() == MSBFIRST);

    if (msb_first) {
        uint8_t mosi_high = static_cast<uint8_t>((data >> 8) & 0xFF);
        uint8_t mosi_low = static_cast<uint8_t>(data & 0xFF);
        uint8_t miso_high = transfer(mosi_high);
        uint8_t miso_low = transfer(mosi_low);
        return (static_cast<uint16_t>(miso_high) << 8) | miso_low;
    } else {
        uint8_t mosi_low = static_cast<uint8_t>(data & 0xFF);
        uint8_t mosi_high = static_cast<uint8_t>((data >> 8) & 0xFF);
        uint8_t miso_low = transfer(mosi_low);
        uint8_t miso_high = transfer(mosi_high);
        return (static_cast<uint16_t>(miso_high) << 8) | miso_low;
    }
}

void SPIBus::transfer(void *buf, size_t count) {
    if (!buf || count == 0) return;
    auto* ptr = static_cast<uint8_t*>(buf);
    for (size_t i = 0; i < count; ++i) {
        ptr[i] = transfer(ptr[i]);
    }
}

std::vector<SpiTransaction> SPIBus::get_transaction_history() const {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    return history_;
}

void SPIBus::clear_history() {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    history_.clear();
}

void SPIBus::add_observer(std::shared_ptr<ISpiObserver> observer) {
    if (!observer) return;
    std::lock_guard<std::mutex> lock(bus_mtx_);
    observers_.push_back(observer);
}

void SPIBus::remove_observer(std::shared_ptr<ISpiObserver> observer) {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    observers_.erase(
        std::remove_if(observers_.begin(), observers_.end(),
            [&](const std::weak_ptr<ISpiObserver>& wp) {
                auto sp = wp.lock();
                return !sp || sp == observer;
            }),
        observers_.end()
    );
}

void SPIBus::notify_observers(const SpiTransaction& tx) {
    std::vector<std::shared_ptr<ISpiObserver>> active;
    {
        std::lock_guard<std::mutex> lock(bus_mtx_);
        for (const auto& wp : observers_) {
            if (auto sp = wp.lock()) {
                active.push_back(sp);
            }
        }
    }

    for (const auto& obs : active) {
        obs->on_spi_transaction(tx);
    }
}

} // namespace toad::sim

