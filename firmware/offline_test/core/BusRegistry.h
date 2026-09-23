#pragma once

#include <cstdint>
#include <memory>
#include <map>
#include <tuple>
#include <string>
#include <mutex>

namespace toad::sim {

class UartBus;
class SPIBus;
class CANBus;

class BusRegistry {
public:
    static BusRegistry& instance();

    // UART Pin-pair registration
    void register_uart(uint32_t rx, uint32_t tx, std::shared_ptr<UartBus> bus);
    std::shared_ptr<UartBus> get_uart(uint32_t rx, uint32_t tx) const;
    std::shared_ptr<UartBus> get_or_create_uart(uint32_t rx, uint32_t tx, uint32_t de = 0);

    // UART Named bus registration (e.g. "RS485_6", "HW_CommsSerial")
    void register_named_uart(const std::string& name, std::shared_ptr<UartBus> bus);
    std::shared_ptr<UartBus> get_named_uart(const std::string& name) const;

    // SPI Pin-triplet registration (mosi, miso, sck)
    void register_spi(uint32_t mosi, uint32_t miso, uint32_t sck, std::shared_ptr<SPIBus> bus);
    std::shared_ptr<SPIBus> get_spi(uint32_t mosi, uint32_t miso, uint32_t sck) const;
    std::shared_ptr<SPIBus> get_or_create_spi(uint32_t mosi, uint32_t miso, uint32_t sck);

    // SPI Named bus registration (e.g. "PT_TC_SPI_1", "PT_TC_SPI_3")
    void register_named_spi(const std::string& name, std::shared_ptr<SPIBus> bus);
    std::shared_ptr<SPIBus> get_named_spi(const std::string& name) const;

    // CAN Pin-pair registration (rx, tx)
    void register_can(uint32_t rx, uint32_t tx, std::shared_ptr<CANBus> bus);
    std::shared_ptr<CANBus> get_can(uint32_t rx, uint32_t tx) const;
    std::shared_ptr<CANBus> get_or_create_can(uint32_t rx, uint32_t tx, uint32_t bitrate = 500000);

    // CAN Named bus registration (e.g. "CAN_TVC", "CAN_FC")
    void register_named_can(const std::string& name, std::shared_ptr<CANBus> bus);
    std::shared_ptr<CANBus> get_named_can(const std::string& name) const;
    std::shared_ptr<CANBus> get_or_create_named_can(const std::string& name, uint32_t bitrate = 500000);

    // Reset all registrations
    void reset();

private:
    BusRegistry() = default;
    ~BusRegistry() = default;
    BusRegistry(const BusRegistry&) = delete;
    BusRegistry& operator=(const BusRegistry&) = delete;

    using PinPair = std::pair<uint32_t, uint32_t>;
    using SpiPinTriplet = std::tuple<uint32_t, uint32_t, uint32_t>;

    mutable std::mutex mtx_;
    std::map<PinPair, std::shared_ptr<UartBus>> uart_buses_;
    std::map<std::string, std::shared_ptr<UartBus>> named_uart_buses_;

    std::map<SpiPinTriplet, std::shared_ptr<SPIBus>> spi_buses_;
    std::map<std::string, std::shared_ptr<SPIBus>> named_spi_buses_;

    std::map<PinPair, std::shared_ptr<CANBus>> can_buses_;
    std::map<std::string, std::shared_ptr<CANBus>> named_can_buses_;
};


} // namespace toad::sim
