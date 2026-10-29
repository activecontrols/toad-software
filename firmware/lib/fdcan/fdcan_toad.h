#pragma once
#include <stdint.h>
#include "api/HardwareCAN.h"
#include "Arduino.h"
#include "stm32h7xx_hal.h"
#include "CommandRouter.h"

class CAN;

// instantiated in fdcan_toad.cpp
extern CAN can_tvc;
extern CAN can_fc;

class CAN : public arduino::HardwareCAN
{
public:
    CAN(uint32_t tx_pin, uint32_t rx_pin);

    enum class error_type_t 
    {
        TX_ERROR,
        RX_ERROR,
        RX_ERROR_PASSIVE 
    };
    
    typedef void (*CAN_error_cbk_t)(error_type_t error_type, uint32_t new_count);

    // Initialize using standard Arduino CanBitRate enum.
    // Conforms to arduino::HardwareCAN interface.
    // Returns true on success, false on failure.
    bool begin(CanBitRate bit_rate) override;

    // Settings for starting a bus. CAN-FD fields will be added here later.
    struct config_t
    {
        CanBitRate nominal_bit_rate = CanBitRate::BR_500k;
        uint32_t mode = FDCAN_MODE_NORMAL; // or FDCAN_MODE_INTERNAL_LOOPBACK for testing
        uint32_t frame_format = FDCAN_FRAME_CLASSIC; // or FDCAN_FRAME_FD_NO_BRS
    };


    // CAN-FD message type
    struct fd_msg_t
    {
        uint32_t id = 0;
        uint8_t len = 0; // number of data bytes, 0-64
        uint8_t data[64] = {0};
    };

    // Returns 1 on success, 0 on error or timeout
    int write_fd(const fd_msg_t& msg);

    // Returns true and fills msg if a frame was waiting, false if not
    bool read_fd(fd_msg_t& msg);

    // Initialize with full control over the settings.
    // Returns true on success, false on failure.
    bool begin(const config_t& config);

    // Sends a known frame and checks that it comes back unchanged.
    // Only passes when the bus was started in FDCAN_MODE_INTERNAL_LOOPBACK or FDCAN_MODE_EXTERNAL_LOOPBACK.
    static void loopback_test(CAN& bus);

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

    struct FDCAN_Handle_Wrapper : public FDCAN_HandleTypeDef
    {
        CAN* obj; // pointer back to the CAN object that owns this peripheral
    };


    FDCAN_Handle_Wrapper* handle(void)
    {
        return &hfdcan;
    }

    static bool init(void)
    {
        bool success = true;
        
        // TODO - print error for each CAN bus that fails
        success = success && can_tvc.begin(CanBitRate::BR_500k);
        success = success && can_fc.begin(CanBitRate::BR_500k);

        if (success)
        {
            CommandRouter::add([]() { CAN::loopback_test(can_tvc); }, 
            "can_loopback_test",
            "Send a frame on can_tvc and check it comes back. Needs a loopback mode.");
        };

        return success;
    }

    
private:
    FDCAN_Handle_Wrapper hfdcan = {0};
    FDCAN_ErrorCountersTypeDef error_counts = {0};
    FDCAN_RxHeaderTypeDef rx_header = {0};
    uint32_t tx_pin = 0;
    uint32_t rx_pin = 0;
    uint8_t rx_data[64] = {0};
    CAN_error_cbk_t error_cbk = nullptr;
    uint32_t frame_format = FDCAN_FRAME_CLASSIC;
};
