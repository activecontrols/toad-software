#pragma once

#include <cstdint>
#include <memory>
#include <map>
#include <string>
#include <mutex>

namespace toad::sim {

class UartBus;

class BusRegistry {
public:
    static BusRegistry& instance();

    // Pin-pair registration
    void register_uart(uint32_t rx, uint32_t tx, std::shared_ptr<UartBus> bus);
    std::shared_ptr<UartBus> get_uart(uint32_t rx, uint32_t tx) const;
    std::shared_ptr<UartBus> get_or_create_uart(uint32_t rx, uint32_t tx, uint32_t de = 0);

    // Named bus registration (e.g. "RS485_6", "HW_CommsSerial")
    void register_named_uart(const std::string& name, std::shared_ptr<UartBus> bus);
    std::shared_ptr<UartBus> get_named_uart(const std::string& name) const;

    // Reset all registrations
    void reset();

private:
    BusRegistry() = default;
    ~BusRegistry() = default;
    BusRegistry(const BusRegistry&) = delete;
    BusRegistry& operator=(const BusRegistry&) = delete;

    using PinPair = std::pair<uint32_t, uint32_t>;
    mutable std::mutex mtx_;
    std::map<PinPair, std::shared_ptr<UartBus>> uart_buses_;
    std::map<std::string, std::shared_ptr<UartBus>> named_uart_buses_;
};

} // namespace toad::sim

