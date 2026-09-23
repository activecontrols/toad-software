#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <string>

namespace toad::sim {

enum class BusDirection {
    FW_TO_BUS, // Transmitted by firmware into the bus
    BUS_TO_FW  // Transmitted by peripheral/harness into firmware
};

inline const char* to_string(BusDirection dir) {
    switch (dir) {
        case BusDirection::FW_TO_BUS: return "FW -> BUS";
        case BusDirection::BUS_TO_FW: return "BUS -> FW";
        default: return "UNKNOWN";
    }
}

struct UartTransaction {
    uint64_t timestamp_us{0};
    BusDirection direction{BusDirection::FW_TO_BUS};
    std::vector<uint8_t> data;
};

class IUartObserver {
public:
    virtual ~IUartObserver() = default;
    virtual void on_uart_transaction(const UartTransaction& tx) = 0;
};

class IUartObservable {
public:
    virtual ~IUartObservable() = default;
    virtual void add_observer(std::shared_ptr<IUartObserver> observer) = 0;
    virtual void remove_observer(std::shared_ptr<IUartObserver> observer) = 0;
};

struct SpiTransaction {
    uint64_t timestamp_us{0};
    uint32_t cs_pin{0};
    std::string device_name;
    std::vector<uint8_t> mosi_data;
    std::vector<uint8_t> miso_data;
};

class ISpiObserver {
public:
    virtual ~ISpiObserver() = default;
    virtual void on_spi_transaction(const SpiTransaction& tx) = 0;
};

class ISpiObservable {
public:
    virtual ~ISpiObservable() = default;
    virtual void add_observer(std::shared_ptr<ISpiObserver> observer) = 0;
    virtual void remove_observer(std::shared_ptr<ISpiObserver> observer) = 0;
};

struct CanTransaction {
    uint64_t timestamp_us{0};
    uint32_t can_id{0};
    bool extended{false};
    bool rtr{false};
    std::string sender_name;
    std::vector<uint8_t> data;
};

class ICanObserver {
public:
    virtual ~ICanObserver() = default;
    virtual void on_can_transaction(const CanTransaction& tx) = 0;
};

class ICanObservable {
public:
    virtual ~ICanObservable() = default;
    virtual void add_observer(std::shared_ptr<ICanObserver> observer) = 0;
    virtual void remove_observer(std::shared_ptr<ICanObserver> observer) = 0;
};

struct BusTransaction {
    uint64_t timestamp_us{0};
    std::vector<uint8_t> tx_data;
    std::vector<uint8_t> rx_data;
    uint32_t channel_or_id{0};
    std::string channel_name;
};

} // namespace toad::sim

