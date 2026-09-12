#pragma once
#include <stdint.h>
#include "api/HardwareCAN.h"



class CAN : arduino::HardwareCAN
{
public:
    CAN(uint32_t tx_pin, uint32_t rx_pin);

    // initialize the FDCAN peripheral for use with the Toad EC
    // returns actual bitrate in bps
    uint32_t begin(uint32_t bit_rate);

    bool begin(CanBitRate bit_rate);

    // get number of elements in the receive fifo
    size_t available(void);

    // get number free positions in the transmit fifo - not part of the arduino API
    uint32_t tx_free_count(void);

    // transmit a data frame in classic CAN mode with an 11 bit ID
    // return true on success, false on error
    int write(CanMsg const & msg);


    // if a message was waiting in the recieve FIFO, this returns true; otherwise returns false
    // returns false if an error occurs
    // 
    CanMsg read(void);

    void end(void);

// private:
    FDCAN_HandleTypeDef hfdcan;
    uint32_t tx_pin;
    uint32_t rx_pin;
    uint8_t rx_data[64];
    FDCAN_RxHeaderTypeDef rx_header;
};