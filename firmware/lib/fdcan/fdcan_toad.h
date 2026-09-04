#pragma once
#include <stdint.h>
#include "stm32h7xx_hal.h"

class CAN
{
public:
    CAN(FDCAN_GlobalTypeDef* instance);

    // initialize the FDCAN peripheral for use with the Toad EC
    void init(void);

    // get number of elements in the receive fifo
    uint32_t rcv_count(void);

    // get number free positions in the transmit fifo
    uint32_t tx_free_count(void);

    // transmit a data frame in classic CAN mode with an 11 bit ID
    // return true on success, false on error
    bool send(uint16_t id, uint32_t data_length, const uint8_t* data);


    // if a message was waiting in the recieve FIFO, this returns true; otherwise returns false
    // returns false if an error occurs
    // 
    bool receive(FDCAN_RxHeaderTypeDef* header, uint8_t* data);

private:
    FDCAN_HandleTypeDef hfdcan;
};