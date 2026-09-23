#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <algorithm>
#include "api/CanMsg.h"

namespace toad::sim {

/**
 * @brief Universal CAN frame representation for CAN 2.0 (up to 8 bytes) and CAN-FD (up to 64 bytes).
 *
 * Provides seamless conversion to and from official arduino::CanMsg.
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

    // Implicit/explicit conversion from arduino::CanMsg
    CanFrame(const arduino::CanMsg& msg, uint64_t timestamp = 0)
        : id(msg.isExtendedId() ? msg.getExtendedId() : msg.getStandardId()),
          extended(msg.isExtendedId()),
          len(msg.data_length),
          timestamp_us(timestamp) {
        if (len > 0) {
            std::memcpy(data, msg.data, len);
        }
    }

    // Conversion to arduino::CanMsg
    arduino::CanMsg to_can_msg() const {
        uint32_t msg_id = extended ? (id | arduino::CanMsg::CAN_EFF_FLAG) : (id & arduino::CanMsg::CAN_SFF_MASK);
        return arduino::CanMsg(msg_id, len, data);
    }

    operator arduino::CanMsg() const {
        return to_can_msg();
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

} // namespace toad::sim

