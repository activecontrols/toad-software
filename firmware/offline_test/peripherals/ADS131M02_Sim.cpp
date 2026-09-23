#include "ADS131M02_Sim.h"
#include "core/VirtualClock.h"
#include "core/SimulatedGPIO.h"
#include "hal_mock/Arduino.h"

namespace toad::sim {

ADS131M02_Sim::ADS131M02_Sim(std::string name, uint32_t drdy_pin)
    : name_(std::move(name)), drdy_pin_(drdy_pin) {
    update_latched_frame();
}

ADS131M02_Sim::~ADS131M02_Sim() {
    stop();
}

void ADS131M02_Sim::start(int priority) {
    if (running_) return;
    running_ = true;
    fiber_ = launch_fiber_with_priority(priority, [this]() {
        this->conversion_loop();
    });
}

void ADS131M02_Sim::stop() {
    if (!running_) return;
    running_ = false;
    VirtualClock::instance().wake_all();
    if (fiber_.joinable()) {
        fiber_.join();
    }
}

void ADS131M02_Sim::join() {
    if (fiber_.joinable()) {
        fiber_.join();
    }
}

void ADS131M02_Sim::set_ch0_raw(int32_t raw_val) {
    ch0_raw_ = raw_val;
    update_latched_frame();
}

int32_t ADS131M02_Sim::get_ch0_raw() const {
    return ch0_raw_;
}

void ADS131M02_Sim::set_ch1_raw(int32_t raw_val) {
    ch1_raw_ = raw_val;
    update_latched_frame();
}

int32_t ADS131M02_Sim::get_ch1_raw() const {
    return ch1_raw_;
}

void ADS131M02_Sim::set_status_reg(uint32_t status) {
    status_reg_ = status;
    update_latched_frame();
}

uint32_t ADS131M02_Sim::get_status_reg() const {
    return status_reg_;
}

void ADS131M02_Sim::set_sample_period_us(uint32_t period_us) {
    sample_period_us_ = period_us;
}

uint32_t ADS131M02_Sim::sample_period_us() const {
    return sample_period_us_;
}

void ADS131M02_Sim::set_drdy_pin(uint32_t pin) {
    drdy_pin_ = pin;
}

uint32_t ADS131M02_Sim::drdy_pin() const {
    return drdy_pin_;
}

void ADS131M02_Sim::on_cs_asserted() {
    std::lock_guard<std::mutex> lock(frame_mtx_);
    byte_idx_ = 0;
}

void ADS131M02_Sim::on_cs_deasserted() {
    std::lock_guard<std::mutex> lock(frame_mtx_);
    byte_idx_ = 0;
}

uint8_t ADS131M02_Sim::transfer_byte(uint8_t mosi_byte) {
    (void)mosi_byte;
    std::lock_guard<std::mutex> lock(frame_mtx_);
    if (byte_idx_ < latched_frame_.size()) {
        return latched_frame_[byte_idx_++];
    }
    return 0xFF;
}

uint16_t ADS131M02_Sim::calculate_crc(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void ADS131M02_Sim::update_latched_frame() {
    std::lock_guard<std::mutex> lock(frame_mtx_);

    // Word 0: Status Register (24 bits)
    latched_frame_[0] = static_cast<uint8_t>((status_reg_ >> 16) & 0xFF);
    latched_frame_[1] = static_cast<uint8_t>((status_reg_ >> 8) & 0xFF);
    latched_frame_[2] = static_cast<uint8_t>((status_reg_ >> 0) & 0xFF);

    // Word 1: Channel 0 24-bit value
    uint32_t ch0_u = static_cast<uint32_t>(ch0_raw_) & 0x00FFFFFF;
    latched_frame_[3] = static_cast<uint8_t>((ch0_u >> 16) & 0xFF);
    latched_frame_[4] = static_cast<uint8_t>((ch0_u >> 8) & 0xFF);
    latched_frame_[5] = static_cast<uint8_t>((ch0_u >> 0) & 0xFF);

    // Word 2: Channel 1 24-bit value
    uint32_t ch1_u = static_cast<uint32_t>(ch1_raw_) & 0x00FFFFFF;
    latched_frame_[6] = static_cast<uint8_t>((ch1_u >> 16) & 0xFF);
    latched_frame_[7] = static_cast<uint8_t>((ch1_u >> 8) & 0xFF);
    latched_frame_[8] = static_cast<uint8_t>((ch1_u >> 0) & 0xFF);

    // Word 3: CRC word (bytes 9..11: 0x00, crc_msb, crc_lsb)
    uint16_t crc = calculate_crc(latched_frame_.data(), 9);
    latched_frame_[9] = 0x00;
    latched_frame_[10] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    latched_frame_[11] = static_cast<uint8_t>(crc & 0xFF);
}

void ADS131M02_Sim::conversion_loop() {
    while (running_) {
        VirtualClock::instance().sleep_for(sample_period_us_);
        if (!running_) break;

        update_latched_frame();

        // Pulse DRDY LOW to notify MCU
        if (drdy_pin_ != NC) {
            SimulatedGPIO::instance().write_pin(drdy_pin_, arduino::LOW);
            VirtualClock::instance().sleep_for(2); // 2 us pulse
            if (!running_) break;
            SimulatedGPIO::instance().write_pin(drdy_pin_, arduino::HIGH);
        }
    }
}

} // namespace toad::sim
