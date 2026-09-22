#pragma once
#include <stdint.h>
#include "api/HardwareCAN.h"
#include "Arduino.h"
#include "stm32h7xx_hal.h"

class CAN : public arduino::HardwareCAN
{
public:
    CAN(uint32_t tx_pin, uint32_t rx_pin);
    ~CAN() override;

    enum class error_type_t 
    {
        TX_ERROR,
        RX_ERROR,
        RX_ERROR_PASSIVE 
    };
    
    typedef void (*CAN_error_cbk_t)(error_type_t error_type, uint32_t new_count);

    // Initialize the FDCAN peripheral with a specific bitrate in bps.
    // Returns actual bitrate achieved in bps.
    float begin(uint32_t bit_rate);

    // Initialize using standard Arduino CanBitRate enum.
    // Conforms to arduino::HardwareCAN interface.
    bool begin(CanBitRate bit_rate) override;

    // Get number of messages waiting in the receive FIFO.
    // Conforms to arduino::HardwareCAN interface.
    size_t available(void) override;

    // Get number of free positions in the transmit buffer (1 if free, 0 if busy).
    uint32_t tx_free_count(void);

    // Transmit a data frame in classic CAN mode with an 11-bit or 29-bit ID.
    // Conforms to arduino::HardwareCAN interface.
    // Returns 1 on success, 0 on error or timeout.
    int write(arduino::CanMsg const & msg) override;

    // Retrieve the next received message from the receive FIFO.
    // Conforms to arduino::HardwareCAN interface.
    // Returns an empty CanMsg if no message was available or an error occurred.
    arduino::CanMsg read(void) override;

    // Disable the FDCAN peripheral.
    // Conforms to arduino::HardwareCAN interface.
    void end(void) override;

    // Register a callback for CAN bus error updates.
    void set_error_cbk(CAN_error_cbk_t error_cbk);
    
    // Internal handler called from interrupt service routine.
    // Note: Must only be called from an interrupt context, without nested interrupts.
    void error_update_from_isr(void);

    // Wait for ongoing transmission in TX buffer 0 to complete.
    // Returns true if buffer is free, false if timed out.
    bool flush(uint32_t timeout_ms = UINT32_MAX);


// private:
    FDCAN_HandleTypeDef hfdcan = {0};
    FDCAN_ErrorCountersTypeDef error_counts = {0};
    FDCAN_RxHeaderTypeDef rx_header = {0};
    uint32_t tx_pin = 0;
    uint32_t rx_pin = 0;
    uint8_t rx_data[64] = {0};
    CAN_error_cbk_t error_cbk = nullptr;
};