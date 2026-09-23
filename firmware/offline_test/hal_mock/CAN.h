#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <algorithm>
#include <memory>
#include <vector>

namespace toad::sim {

/**
 * @brief Universal CAN frame representation for CAN 2.0 (up to 8 bytes) and CAN-FD (up to 64 bytes).
 */
struct CanFrame {
    uint32_t id{0};           ///< 11-bit standard or 29-bit extended identifier
    bool extended{false};     ///< False for 11-bit standard, True for 29-bit extended
    bool rtr{false};          ///< Remote Transmission Request
    uint8_t len{0};           ///< Payload length (0-64 bytes)
    uint8_t data[64]{0};      ///< Frame payload bytes
    uint64_t timestamp_us{0}; ///< Virtual clock timestamp (us) when frame was sent

    CanFrame() = default;

    CanFrame(uint32_t frame_id, const uint8_t* payload, uint8_t payload_len, bool ext = false, bool is_rtr = false)
        : id(frame_id), extended(ext), rtr(is_rtr), len(std::min<uint8_t>(payload_len, 64)) {
        if (payload && len > 0) {
            std::memcpy(data, payload, len);
        }
    }

    bool operator==(const CanFrame& other) const {
        if (id != other.id || extended != other.extended || rtr != other.rtr || len != other.len) {
            return false;
        }
        return std::memcmp(data, other.data, len) == 0;
    }

    bool operator!=(const CanFrame& other) const {
        return !(*this == other);
    }
};

class CANBus;

} // namespace toad::sim

/**
 * @brief Layer 1 firmware-facing CAN controller class.
 *
 * Implements standard embedded Arduino/STM32-style CAN interface.
 * Forwards write/read operations down to Layer 2 simulation fabric (CANBus).
 */
class CANClass {
public:
    CANClass(uint32_t rx_pin, uint32_t tx_pin);
    explicit CANClass(std::string bus_name);
    ~CANClass() = default;

    CANClass(const CANClass&) = default;
    CANClass& operator=(const CANClass&) = default;

    bool begin(uint32_t baud_rate = 500000);
    void end();

    bool write(const toad::sim::CanFrame& frame);
    bool write(uint32_t id, const uint8_t* data, uint8_t len, bool extended = false, bool rtr = false);

    bool read(toad::sim::CanFrame& frame);
    int available();

    void set_filter(uint32_t id, uint32_t mask = 0x7FF);

    std::shared_ptr<toad::sim::CANBus> backend_bus() const;

private:
    void resolve_bus();

    uint32_t rx_pin_{0};
    uint32_t tx_pin_{0};
    std::string bus_name_;
    std::shared_ptr<toad::sim::CANBus> backend_bus_{nullptr};
    uint32_t filter_id_{0};
    uint32_t filter_mask_{0};
    bool filter_enabled_{false};
};

extern CANClass CAN_TVC;
extern CANClass CAN_FC;
extern CANClass CAN;

