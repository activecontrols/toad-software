#pragma once
#include <stdint.h>

// todo - confirm which can bus the actuators are on
namespace CAN1
{
    // initialize the FDCAN1 peripheral for use with the Toad EC
    void init(void);

    // get number of elements in the receive fifo
    uint32_t rcv_count(void);

    // get number free positions in the transmit fifo
    uint32_t tx_free_count(void);

    // transmit a data frame in classic CAN mode with an 11 bit ID
    // return true on success, false on error
    bool send(uint16_t id, )
    
};