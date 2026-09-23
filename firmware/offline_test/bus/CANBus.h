#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>

#include "hal_mock/CAN.h"
#include "bus/ITransactionObservable.h"
#include "core/VirtualClock.h"

namespace toad::sim {

class ICANDevice;

/**
 * @brief Layer 2 Multi-Drop CAN Bus Simulation Fabric.
 *
 * Models physical copper CAN network:
 * - Broadcasts messages to all subscribed ICANDevice nodes and MCU RX queue.
 * - Advances virtual time based on bitrate and frame payload length.
 * - Implements ICanObservable and ITransactionObservable for logging and packet inspection.
 */
class CANBus : public ICanObservable, public std::enable_shared_from_this<CANBus> {
public:
    explicit CANBus(std::string name = "CAN_BUS", uint32_t bitrate = 500000);
    virtual ~CANBus() = default;

    const std::string& name() const { return name_; }
    uint32_t bitrate() const { return bitrate_; }
    void set_bitrate(uint32_t bitrate) { bitrate_ = bitrate; }

    void set_zero_latency(bool zero_latency) { zero_latency_ = zero_latency; }
    bool zero_latency() const { return zero_latency_; }

    // Subscriber management
    void subscribe(const std::shared_ptr<ICANDevice>& device);
    void unsubscribe(const std::shared_ptr<ICANDevice>& device);
    std::vector<std::shared_ptr<ICANDevice>> subscribers() const;

    // Multi-drop broadcast
    bool broadcast(const CanFrame& frame, const ICANDevice* sender = nullptr);

    // MCU Firmware interface
    bool transmit_from_firmware(const CanFrame& frame);
    bool read_to_firmware(CanFrame& frame);
    int firmware_available() const;
    void clear_firmware_rx();

    // Timing calculation
    uint64_t calculate_transmission_time_us(const CanFrame& frame) const;

    // Observability (ICanObservable)
    void add_observer(std::shared_ptr<ICanObserver> observer) override;
    void remove_observer(std::shared_ptr<ICanObserver> observer) override;

    const std::vector<CanTransaction>& transaction_history() const { return history_; }
    void clear_history() { history_.clear(); }

    // Universal BusTransaction inspection
    std::vector<BusTransaction> get_bus_transactions() const;

private:
    std::string name_;
    uint32_t bitrate_{500000};
    bool zero_latency_{false};

    mutable std::mutex mtx_;
    std::vector<std::shared_ptr<ICANDevice>> devices_;
    std::deque<CanFrame> mcu_rx_queue_;
    std::vector<CanTransaction> history_;
    std::vector<std::shared_ptr<ICanObserver>> observers_;
};

} // namespace toad::sim

