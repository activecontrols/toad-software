#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ITransactionObservable.h"
#include "peripherals/ISPIDevice.h"
#include "hal_mock/pins_arduino.h"
#include "api/HardwareSPI.h"

namespace toad::sim {

struct DeviceRegistration {
    std::shared_ptr<ISPIDevice> device;
    bool active_low{true};
};

class SPIBus : public ISpiObservable {
public:
    explicit SPIBus(std::string bus_name = "SPI_BUS",
                    uint32_t mosi_pin = 0,
                    uint32_t miso_pin = 0,
                    uint32_t sck_pin = 0);
    ~SPIBus() override;

    // Bus identification & pin mapping
    const std::string& name() const { return bus_name_; }
    void set_name(const std::string& name) { bus_name_ = name; }

    uint32_t mosi_pin() const { return mosi_pin_; }
    uint32_t miso_pin() const { return miso_pin_; }
    uint32_t sck_pin() const { return sck_pin_; }

    // Latency simulation toggle
    bool zero_latency() const { return zero_latency_; }
    void set_zero_latency(bool enabled) { zero_latency_ = enabled; }

    // Device Attachment & Registration by CS pin
    void register_device(uint32_t cs_pin, std::shared_ptr<ISPIDevice> device, bool active_low = true);
    void unregister_device(uint32_t cs_pin);
    std::shared_ptr<ISPIDevice> get_device(uint32_t cs_pin) const;

    // CS State & Bus Contention Inspection
    uint32_t active_cs_pin() const;
    std::shared_ptr<ISPIDevice> active_device() const;
    bool has_bus_contention() const;
    size_t contention_count() const;
    void reset_contention();

    // Transaction Management
    void begin_transaction(arduino::SPISettings settings);
    void end_transaction();
    const arduino::SPISettings& current_settings() const { return current_settings_; }

    // Data Transfers
    uint8_t transfer(uint8_t mosi_byte);
    uint16_t transfer16(uint16_t data);
    void transfer(void *buf, size_t count);

    // Transaction History & Observability
    std::vector<SpiTransaction> get_transaction_history() const;
    void clear_history();

    void add_observer(std::shared_ptr<ISpiObserver> observer) override;
    void remove_observer(std::shared_ptr<ISpiObserver> observer) override;

private:
    void on_cs_pin_changed(uint32_t pin, PinStatus new_status, PinStatus old_status);
    void update_cs_state();
    uint64_t calculate_byte_duration_us() const;
    void notify_observers(const SpiTransaction& tx);

    std::string bus_name_;
    uint32_t mosi_pin_{0};
    uint32_t miso_pin_{0};
    uint32_t sck_pin_{0};

    bool zero_latency_{false};
    arduino::SPISettings current_settings_{4000000, MSBFIRST, arduino::SPI_MODE0};

    mutable std::mutex bus_mtx_;
    std::map<uint32_t, DeviceRegistration> devices_;

    uint32_t active_cs_pin_{NC};
    std::shared_ptr<ISPIDevice> active_device_{nullptr};

    bool bus_contention_{false};
    size_t contention_count_{0};

    bool in_transaction_{false};
    SpiTransaction current_tx_;
    std::vector<SpiTransaction> history_;
    std::vector<std::weak_ptr<ISpiObserver>> observers_;
};

} // namespace toad::sim
