#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include "bus/CANBus.h"


namespace toad::sim {

/**
 * @brief Cross-process SocketCAN interface placeholder.
 *
 * In Phase 1 (SITL in-process), wraps a local CANBus instance.
 * In Phase 2, connects to Linux SocketCAN (e.g. vcan0) to bridge across processes to the Flight Controller.
 */
class SystemCAN {
public:
    explicit SystemCAN(std::string interface_name = "vcan0", uint32_t bitrate = 500000);
    ~SystemCAN() = default;

    bool open();
    void close();
    bool is_open() const { return is_open_; }

    const std::string& interface_name() const { return interface_name_; }

    bool send(const CanFrame& frame);
    bool receive(CanFrame& frame);
    int available();

    std::shared_ptr<CANBus> local_bus() const { return local_bus_; }

private:
    std::string interface_name_;
    uint32_t bitrate_{500000};
    bool is_open_{false};
    std::shared_ptr<CANBus> local_bus_{nullptr};
};

} // namespace toad::sim

