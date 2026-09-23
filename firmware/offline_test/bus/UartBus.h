#pragma once

#include <cstdint>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "ITransactionObservable.h"

namespace toad::sim {

class UartBus : public IUartObservable, public std::enable_shared_from_this<UartBus> {
public:
    explicit UartBus(unsigned long baud = 115200, std::string bus_name = "UART_BUS");
    ~UartBus() override = default;

    // Bus identification & configuration
    const std::string& name() const { return bus_name_; }
    void set_name(const std::string& name) { bus_name_ = name; }

    unsigned long baud_rate() const { return baud_rate_; }
    void set_baud_rate(unsigned long baud);

    bool zero_latency() const { return zero_latency_; }
    void set_zero_latency(bool enabled) { zero_latency_ = enabled; }

    // --- Firmware Interface (called by Uart mock) ---
    virtual void write_from_firmware(uint8_t byte);
    virtual void write_from_firmware(const uint8_t* buffer, size_t size);

    virtual int read_for_firmware();
    virtual int peek_for_firmware() const;
    virtual size_t available_for_firmware() const;
    virtual void flush_firmware();

    // --- Simulation / Peripheral / Harness Interface ---
    virtual void write_to_firmware(uint8_t byte);
    virtual void write_to_firmware(const uint8_t* buffer, size_t size);
    virtual void write_to_firmware(const std::string& str);

    virtual int read_from_firmware();
    virtual size_t available_from_firmware() const;

    // --- Sim-Side Peeking & Inspection (Non-Destructive) ---
    // Peek at the queues without removing elements
    int peek_fw_tx() const;
    int peek_bus_tx() const;
    std::vector<uint8_t> peek_all_fw_tx() const;
    std::vector<uint8_t> peek_all_bus_tx() const;

    // History of transactions
    std::vector<UartTransaction> get_transaction_history() const;
    void clear_history();

    // --- Observer Management ---
    void add_observer(std::shared_ptr<IUartObserver> observer) override;
    void remove_observer(std::shared_ptr<IUartObserver> observer) override;

    // Helper: calculate transmission duration in virtual microseconds for N bytes
    uint64_t calculate_duration_us(size_t byte_count) const;

private:
    void notify_observers(const UartTransaction& tx);

    std::string bus_name_;
    unsigned long baud_rate_{115200};
    bool zero_latency_{false};

    mutable std::mutex mtx_;
    std::deque<uint8_t> fw_to_bus_queue_; // Outgoing from firmware
    std::deque<uint8_t> bus_to_fw_queue_; // Incoming to firmware

    std::vector<UartTransaction> history_;
    std::vector<std::weak_ptr<IUartObserver>> observers_;
};

} // namespace toad::sim
