#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "UartBus.h"
#include "peripherals/IRS485Device.h"
#include "hal_mock/pins_arduino.h"

namespace toad::sim {

class RS485Mux : public UartBus {
public:
    explicit RS485Mux(unsigned long baud = 115200,
                      std::string bus_name = "RS485_MUX",
                      uint32_t de_pin = NC);
    ~RS485Mux() override;

    // Device Attachment & Registration by SEL pin
    void register_device(uint32_t sel_pin, std::shared_ptr<IRS485Device> device);
    void unregister_device(uint32_t sel_pin);
    std::shared_ptr<IRS485Device> get_device(uint32_t sel_pin) const;

    // Pin inspection & Contention State
    uint32_t de_pin() const { return de_pin_; }
    void set_de_pin(uint32_t pin) { de_pin_ = pin; }

    uint32_t active_sel_pin() const;
    std::shared_ptr<IRS485Device> active_device() const;

    bool has_bus_contention() const;
    size_t contention_count() const;
    void reset_contention();

    // Hook called when firmware writes bytes onto this bus
    void write_from_firmware(const uint8_t* buffer, size_t size) override;
    void write_from_firmware(uint8_t byte) override;

    // Called by attached devices when they reply onto the RS-485 bus
    void receive_from_device(IRS485Device* device, const uint8_t* buffer, size_t size);

private:
    void on_sel_pin_changed(uint32_t pin, arduino::PinStatus new_status, arduino::PinStatus old_status);
    void update_mux_state();

    uint32_t de_pin_{NC};

    mutable std::mutex mux_mtx_;
    std::map<uint32_t, std::shared_ptr<IRS485Device>> devices_;

    uint32_t active_sel_pin_{NC};
    std::shared_ptr<IRS485Device> active_device_{nullptr};

    bool bus_contention_{false};
    size_t contention_count_{0};
};

} // namespace toad::sim
