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

    // Settings for starting a bus.
    struct config_t
    {
        CanBitRate nominal_bit_rate = CanBitRate::BR_500k; // arbitration phase (and the whole frame without BRS)
        uint32_t mode = FDCAN_MODE_NORMAL; // or FDCAN_MODE_INTERNAL_LOOPBACK / FDCAN_MODE_EXTERNAL_LOOPBACK for testing
        uint32_t frame_format = FDCAN_FRAME_CLASSIC; // or FDCAN_FRAME_FD_NO_BRS / FDCAN_FRAME_FD_BRS
        uint32_t data_bit_rate = 8'000'000; // data phase in bit/s, only used with FDCAN_FRAME_FD_BRS
    };


    // CAN-FD message type
    struct fd_msg_t
    {
        uint32_t id = 0;
        uint8_t len = 0; // number of data bytes, 0-64
        uint8_t data[64] = {0};
    };

    // Transmit a CAN-FD frame. The bus must have been started with an FD frame_format.
    // On an FDCAN_FRAME_FD_BRS bus the data phase is sent at data_bit_rate.
    // Lengths that are not a valid FD size are padded with zeros to the next valid size (e.g. 10 -> 12).
    // Returns 1 on success, 0 on error or timeout.
    int write_fd(const fd_msg_t& msg);

    // Take the next received frame (classic or FD) out of the receive FIFO.
    // Returns true and fills msg if a frame was waiting, false if not.
    bool read_fd(fd_msg_t& msg);

    // Initialize with full control over the settings.
    // Returns true on success, false on failure.
    bool begin(const config_t& config);

    // Sends a known frame and checks that it comes back unchanged.
    // Only passes when the bus was started in FDCAN_MODE_INTERNAL_LOOPBACK or FDCAN_MODE_EXTERNAL_LOOPBACK.
    static void loopback_test(CAN& bus);

    // Sends FD frames of 8, 10, 20 and 64 bytes and checks each comes back with the right length,
    // data and zero padding. Needs an FD frame_format and a loopback mode.
    static void loopback_test_fd(CAN& bus);

    // Sends FD frames of 8, 10, 20 and 64 bytes from tx_bus and checks each arrives on rx_bus with the right
    // length, data and zero padding, then prints both buses' error counters and protocol status.
    // Needs the two buses wired to each other, both in normal mode with an FD frame_format.
    static void node_test(CAN& tx_bus, CAN& rx_bus);

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
        // TEST ONLY: two-node test. can_tvc and can_fc are wired to each other (CANH to CANH, CANL to CANL,
        // 120 ohm termination at each end) with nothing else on that wire. The TVC actuators and valve motors
        // must be unplugged, because they only speak classic CAN and would reject FD frames with error frames.
        // Both buses run in normal mode with FD frames and bit rate switching, so each one acknowledges the
        // other's frames like a real second node. Both buses must use the same nominal and data bit rates.
        CAN::config_t tvc_config;
        tvc_config.mode = FDCAN_MODE_NORMAL;
        tvc_config.frame_format = FDCAN_FRAME_FD_BRS;
        tvc_config.data_bit_rate = 8'000'000; // if the node test fails at 8 Mbit/s, try 2'000'000 on both buses
        success = success && can_tvc.begin(tvc_config);

        CAN::config_t fc_config;
        fc_config.mode = FDCAN_MODE_NORMAL;
        fc_config.frame_format = FDCAN_FRAME_FD_BRS;
        fc_config.data_bit_rate = 8'000'000;
        success = success && can_fc.begin(fc_config);

        if (success)
        {
            CommandRouter::add([]() { CAN::loopback_test(can_tvc); }, 
            "can_loopback_test",
            "Send a frame on can_tvc and check it comes back. Needs a loopback mode.");
            CommandRouter::add([]() { CAN::loopback_test_fd(can_tvc); },
            "can_fd_loopback_test",
            "Send FD frames of 8, 10, 20 and 64 bytes on can_tvc and check they come back. Needs an FD frame_format and a loopback mode.");
            CommandRouter::add([]() { CAN::node_test(can_tvc, can_fc); },
            "can_node_test_tvc_to_fc",
            "TEST ONLY: send FD frames from can_tvc and check can_fc receives them. Needs the two buses wired together.");
            CommandRouter::add([]() { CAN::node_test(can_fc, can_tvc); },
            "can_node_test_fc_to_tvc",
            "TEST ONLY: send FD frames from can_fc and check can_tvc receives them. Needs the two buses wired together.");
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
