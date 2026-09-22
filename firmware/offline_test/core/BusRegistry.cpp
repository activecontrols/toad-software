#include "BusRegistry.h"
#include "bus/UartBus.h"
#include "bus/SPIBus.h"

namespace toad::sim {

BusRegistry& BusRegistry::instance() {
    static BusRegistry instance;
    return instance;
}

void BusRegistry::register_uart(uint32_t rx, uint32_t tx, std::shared_ptr<UartBus> bus) {
    std::lock_guard<std::mutex> lock(mtx_);
    uart_buses_[{rx, tx}] = bus;
}

std::shared_ptr<UartBus> BusRegistry::get_uart(uint32_t rx, uint32_t tx) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = uart_buses_.find({rx, tx});
    if (it != uart_buses_.end()) {
        return it->second;
    }
    return nullptr;
}

std::shared_ptr<UartBus> BusRegistry::get_or_create_uart(uint32_t rx, uint32_t tx, uint32_t de) {
    (void)de;
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = uart_buses_.find({rx, tx});
    if (it != uart_buses_.end()) {
        return it->second;
    }

    std::string name = "UART_RX" + std::to_string(rx) + "_TX" + std::to_string(tx);
    auto new_bus = std::make_shared<UartBus>(115200, name);
    uart_buses_[{rx, tx}] = new_bus;
    return new_bus;
}

void BusRegistry::register_named_uart(const std::string& name, std::shared_ptr<UartBus> bus) {
    std::lock_guard<std::mutex> lock(mtx_);
    named_uart_buses_[name] = bus;
}

std::shared_ptr<UartBus> BusRegistry::get_named_uart(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = named_uart_buses_.find(name);
    if (it != named_uart_buses_.end()) {
        return it->second;
    }
    return nullptr;
}

void BusRegistry::register_spi(uint32_t mosi, uint32_t miso, uint32_t sck, std::shared_ptr<SPIBus> bus) {
    std::lock_guard<std::mutex> lock(mtx_);
    spi_buses_[{mosi, miso, sck}] = bus;
}

std::shared_ptr<SPIBus> BusRegistry::get_spi(uint32_t mosi, uint32_t miso, uint32_t sck) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = spi_buses_.find({mosi, miso, sck});
    if (it != spi_buses_.end()) {
        return it->second;
    }
    return nullptr;
}

std::shared_ptr<SPIBus> BusRegistry::get_or_create_spi(uint32_t mosi, uint32_t miso, uint32_t sck) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = spi_buses_.find({mosi, miso, sck});
    if (it != spi_buses_.end()) {
        return it->second;
    }

    std::string name = "SPI_MOSI" + std::to_string(mosi) + "_MISO" + std::to_string(miso) + "_SCK" + std::to_string(sck);
    auto new_bus = std::make_shared<SPIBus>(name, mosi, miso, sck);
    spi_buses_[{mosi, miso, sck}] = new_bus;
    return new_bus;
}

void BusRegistry::register_named_spi(const std::string& name, std::shared_ptr<SPIBus> bus) {
    std::lock_guard<std::mutex> lock(mtx_);
    named_spi_buses_[name] = bus;
}

std::shared_ptr<SPIBus> BusRegistry::get_named_spi(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = named_spi_buses_.find(name);
    if (it != named_spi_buses_.end()) {
        return it->second;
    }
    return nullptr;
}

void BusRegistry::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    uart_buses_.clear();
    named_uart_buses_.clear();
    spi_buses_.clear();
    named_spi_buses_.clear();
}

} // namespace toad::sim
