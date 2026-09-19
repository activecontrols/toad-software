#pragma once
#include <stdint.h>
#include "api/HardwareCAN.h"
#include <memory>
#include "Arduino.h"

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

    // initialize the FDCAN peripheral for use with the Toad EC
    // returns actual bitrate in bps
    float begin(uint32_t bit_rate);

    bool begin(CanBitRate bit_rate) override;

    // get number of elements in the receive fifo
    size_t available(void) override;

    // get number free positions in the transmit fifo - not part of the arduino API
    uint32_t tx_free_count(void);

    // transmit a data frame in classic CAN mode with an 11 bit ID
    // return true on success, false on error
    int write(arduino::CanMsg const & msg) override;


    // if a message was waiting in the recieve FIFO, this returns true; otherwise returns false
    // returns false if an error occurs
    arduino::CanMsg read(void) override;

    void end(void) override;

    void set_error_cbk(CAN_error_cbk_t error_cbk);
    
    // note: must only be called from an interrupt context, without nested interrupts
    void error_update_from_isr(void);

    // wait for all tx to complete
    // returns true if timed out
    bool flush(uint32_t timeout_ms = UINT32_MAX);


// private:
    struct HAL;
    std::unique_ptr<HAL> hal;
    uint32_t tx_pin;
    uint32_t rx_pin;
    uint8_t rx_data[64];
    CAN_error_cbk_t error_cbk = nullptr;
};