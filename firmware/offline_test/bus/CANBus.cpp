#include "CANBus.h"
#include "peripherals/ICANDevice.h"
#include "core/VirtualClock.h"

namespace toad::sim {

void FunctionalCANDevice::transmit(const CanFrame& frame) {
    if (auto b = bus_.lock()) {
        b->broadcast(frame, this);
    }
}

CANBus::CANBus(std::string name, uint32_t bitrate)
    : name_(std::move(name)), bitrate_(bitrate) {}

void CANBus::subscribe(const std::shared_ptr<ICANDevice>& device) {
    if (!device) return;
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& dev : devices_) {
        if (dev == device) return;
    }
    devices_.push_back(device);
    device->set_bus(shared_from_this());
}

void CANBus::unsubscribe(const std::shared_ptr<ICANDevice>& device) {
    if (!device) return;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto it = devices_.begin(); it != devices_.end(); ++it) {
        if (*it == device) {
            devices_.erase(it);
            break;
        }
    }
}

std::vector<std::shared_ptr<ICANDevice>> CANBus::subscribers() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return devices_;
}

uint64_t CANBus::calculate_transmission_time_us(const CanFrame& frame) const {
    if (bitrate_ == 0) return 0;

    // Standard CAN 2.0 frame:
    // SOF (1) + Identifier (11 or 29) + Control/DLC (6) + Data (8*len) + CRC (15) + Delim/ACK/EOF (10) + Intermission (3)
    uint32_t bit_count = frame.extended ? (67 + 8 * frame.len) : (47 + 8 * frame.len);

    // Approximate bit stuffing overhead (~15%)
    bit_count = bit_count * 115 / 100;

    // Total microseconds = (bit_count * 1,000,000) / bitrate
    uint64_t time_us = (static_cast<uint64_t>(bit_count) * 1000000ULL) / bitrate_;
    return time_us > 0 ? time_us : 1;
}

bool CANBus::broadcast(const CanFrame& frame, const ICANDevice* sender) {
    // Model transmission time across the physical bus
    if (!zero_latency_) {
        uint64_t tx_time = calculate_transmission_time_us(frame);
        if (tx_time > 0) {
            VirtualClock::instance().sleep_for(tx_time);
        }
    }

    CanFrame stamped_frame = frame;
    stamped_frame.timestamp_us = VirtualClock::instance().now_us();

    std::vector<std::shared_ptr<ICANDevice>> targets;
    std::vector<std::shared_ptr<ICanObserver>> observer_list;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        targets = devices_;
        observer_list = observers_;

        // If transmitted by an external peripheral node, push into MCU firmware RX queue
        if (sender != nullptr) {
            mcu_rx_queue_.push_back(stamped_frame);
        }

        // Record in transaction history
        CanTransaction tx;
        tx.timestamp_us = stamped_frame.timestamp_us;
        tx.can_id = frame.id;
        tx.extended = frame.extended;
        tx.rtr = frame.rtr;
        tx.sender_name = sender ? sender->name() : "MCU_Firmware";
        tx.data.assign(frame.data, frame.data + frame.len);
        history_.push_back(tx);
    }

    // Deliver to all other subscribed peripheral nodes
    for (const auto& dev : targets) {
        if (dev.get() != sender) {
            dev->enqueue_frame(stamped_frame);
        }
    }

    // Notify observers
    CanTransaction notify_tx;
    notify_tx.timestamp_us = stamped_frame.timestamp_us;
    notify_tx.can_id = frame.id;
    notify_tx.extended = frame.extended;
    notify_tx.rtr = frame.rtr;
    notify_tx.sender_name = sender ? sender->name() : "MCU_Firmware";
    notify_tx.data.assign(frame.data, frame.data + frame.len);

    for (const auto& obs : observer_list) {
        if (obs) obs->on_can_transaction(notify_tx);
    }

    return true;
}

bool CANBus::transmit_from_firmware(const CanFrame& frame) {
    return broadcast(frame, nullptr);
}

bool CANBus::read_to_firmware(CanFrame& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (mcu_rx_queue_.empty()) {
        return false;
    }
    frame = mcu_rx_queue_.front();
    mcu_rx_queue_.pop_front();
    return true;
}

int CANBus::firmware_available() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<int>(mcu_rx_queue_.size());
}

void CANBus::clear_firmware_rx() {
    std::lock_guard<std::mutex> lock(mtx_);
    mcu_rx_queue_.clear();
}

void CANBus::add_observer(std::shared_ptr<ICanObserver> observer) {
    if (!observer) return;
    std::lock_guard<std::mutex> lock(mtx_);
    observers_.push_back(observer);
}

void CANBus::remove_observer(std::shared_ptr<ICanObserver> observer) {
    if (!observer) return;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto it = observers_.begin(); it != observers_.end(); ++it) {
        if (*it == observer) {
            observers_.erase(it);
            break;
        }
    }
}

std::vector<BusTransaction> CANBus::get_bus_transactions() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<BusTransaction> result;
    result.reserve(history_.size());
    for (const auto& tx : history_) {
        BusTransaction btx;
        btx.timestamp_us = tx.timestamp_us;
        btx.channel_or_id = tx.can_id;
        btx.channel_name = tx.sender_name;
        btx.tx_data = tx.data;
        result.push_back(btx);
    }
    return result;
}

} // namespace toad::sim

