#include "UartBus.h"
#include "UartSnooper.h"
#include "core/VirtualClock.h"
#include <algorithm>

namespace toad::sim {

UartBus::UartBus(unsigned long baud, std::string bus_name)
    : bus_name_(std::move(bus_name)), baud_rate_(baud) {}

void UartBus::set_baud_rate(unsigned long baud) {
    std::lock_guard<std::mutex> lock(mtx_);
    baud_rate_ = baud;
}

uint64_t UartBus::calculate_duration_us(size_t byte_count) const {
    if (baud_rate_ == 0 || zero_latency_) return 0;
    // Standard 8N1 serial: 1 start bit + 8 data bits + 1 stop bit = 10 bits per byte
    return (static_cast<uint64_t>(byte_count) * 10ULL * 1000000ULL) / static_cast<uint64_t>(baud_rate_);
}

void UartBus::write_from_firmware(uint8_t byte) {
    write_from_firmware(&byte, 1);
}

void UartBus::write_from_firmware(const uint8_t* buffer, size_t size) {
    if (!buffer || size == 0) return;

    // Simulate baud rate serial wire delay in virtual time
    uint64_t duration_us = calculate_duration_us(size);
    if (duration_us > 0) {
        VirtualClock::instance().sleep_for(duration_us);
    }

    UartTransaction tx;
    tx.timestamp_us = VirtualClock::instance().now_us();
    tx.direction = BusDirection::FW_TO_BUS;
    tx.data.assign(buffer, buffer + size);

    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (size_t i = 0; i < size; ++i) {
            fw_to_bus_queue_.push_back(buffer[i]);
        }
        history_.push_back(tx);
    }

    notify_observers(tx);
}

int UartBus::read_for_firmware() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (bus_to_fw_queue_.empty()) return -1;
    uint8_t byte = bus_to_fw_queue_.front();
    bus_to_fw_queue_.pop_front();
    return byte;
}

int UartBus::peek_for_firmware() const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (bus_to_fw_queue_.empty()) return -1;
    return bus_to_fw_queue_.front();
}

size_t UartBus::available_for_firmware() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return bus_to_fw_queue_.size();
}

void UartBus::flush_firmware() {
    // In cooperative virtual time, write_from_firmware already waited for transmission delay.
}

void UartBus::write_to_firmware(uint8_t byte) {
    write_to_firmware(&byte, 1);
}

void UartBus::write_to_firmware(const uint8_t* buffer, size_t size) {
    if (!buffer || size == 0) return;

    UartTransaction tx;
    tx.timestamp_us = VirtualClock::instance().now_us();
    tx.direction = BusDirection::BUS_TO_FW;
    tx.data.assign(buffer, buffer + size);

    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (size_t i = 0; i < size; ++i) {
            bus_to_fw_queue_.push_back(buffer[i]);
        }
        history_.push_back(tx);
    }

    notify_observers(tx);
}

void UartBus::write_to_firmware(const std::string& str) {
    write_to_firmware(reinterpret_cast<const uint8_t*>(str.data()), str.size());
}

int UartBus::read_from_firmware() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fw_to_bus_queue_.empty()) return -1;
    uint8_t byte = fw_to_bus_queue_.front();
    fw_to_bus_queue_.pop_front();
    return byte;
}

size_t UartBus::available_from_firmware() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return fw_to_bus_queue_.size();
}

int UartBus::peek_fw_tx() const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fw_to_bus_queue_.empty()) return -1;
    return fw_to_bus_queue_.front();
}

int UartBus::peek_bus_tx() const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (bus_to_fw_queue_.empty()) return -1;
    return bus_to_fw_queue_.front();
}

std::vector<uint8_t> UartBus::peek_all_fw_tx() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return {fw_to_bus_queue_.begin(), fw_to_bus_queue_.end()};
}

std::vector<uint8_t> UartBus::peek_all_bus_tx() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return {bus_to_fw_queue_.begin(), bus_to_fw_queue_.end()};
}

std::vector<UartTransaction> UartBus::get_transaction_history() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return history_;
}

void UartBus::clear_history() {
    std::lock_guard<std::mutex> lock(mtx_);
    history_.clear();
}

void UartBus::add_observer(std::shared_ptr<IUartObserver> observer) {
    if (!observer) return;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        observers_.push_back(observer);
    }
    try {
        if (auto snooper = std::dynamic_pointer_cast<UartSnooper>(observer)) {
            snooper->attach_bus(shared_from_this());
        }
    } catch (const std::bad_weak_ptr&) {
        // Not managed by shared_ptr
    }
}

void UartBus::remove_observer(std::shared_ptr<IUartObserver> observer) {
    std::lock_guard<std::mutex> lock(mtx_);
    observers_.erase(
        std::remove_if(observers_.begin(), observers_.end(),
            [&](const std::weak_ptr<IUartObserver>& wp) {
                auto sp = wp.lock();
                return !sp || sp == observer;
            }),
        observers_.end()
    );
}

void UartBus::notify_observers(const UartTransaction& tx) {
    std::vector<std::shared_ptr<IUartObserver>> active_observers;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (auto it = observers_.begin(); it != observers_.end(); ) {
            if (auto sp = it->lock()) {
                active_observers.push_back(sp);
                ++it;
            } else {
                it = observers_.erase(it);
            }
        }
    }

    for (const auto& obs : active_observers) {
        obs->on_uart_transaction(tx);
    }
}

} // namespace toad::sim

