#include "fdcan_toad.h"

#include "Arduino.h"

#include "CommsSerial.h"

using namespace arduino;

/* enable GPIO clock and configure pin (must pass a bit mask for which pin(s) to configure on the specific port) */
static void CAN_init_gpio_dynamic(uint32_t pin, const PinMap pin_map[]) {

  PinName pin_name = digitalPinToPinName(pin);

  // jhillman: I confirmed this enables GPIO clock in the RCC, also sets GPIO speed to very high
  pin_function(pin_name, pinmap_function(pin_name, pin_map));
}

CAN::CAN(uint32_t _tx_pin, uint32_t _rx_pin) {
  this->tx_pin = _tx_pin;
  this->rx_pin = _rx_pin;
  this->hfdcan = {0};
}

// jhillman adapted from
// https://github.com/STMicroelectronics/STM32CubeH7/blob/master/Projects/STM32H743I-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking/Src/main.c
bool CAN::begin(CanBitRate _bitrate) {
  uint32_t bit_rate = (uint32_t)_bitrate;

  FDCAN_GlobalTypeDef *inst_1 =
      static_cast<FDCAN_GlobalTypeDef *>(pinmap_find_peripheral(digitalPinToPinName(tx_pin), PinMap_CAN_TD));
  FDCAN_GlobalTypeDef *inst_2 =
      static_cast<FDCAN_GlobalTypeDef *>(pinmap_find_peripheral(digitalPinToPinName(rx_pin), PinMap_CAN_RD));

  if ((inst_1 != inst_2) || inst_1 == NP) {
    CommsSerial.println("Error: CAN has conflicting pins");
    return false;
  }

  this->hfdcan.Instance = inst_1;

  if (bit_rate == 0U || bit_rate > 1'000'000U) {
    // not allowed for normal CAN operation
    CommsSerial.println("Error: invalid CAN bit rate supplied");
    return false;
  }

  const uint32_t can_ker_ck = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

  if (can_ker_ck != 120'000'000) {
    // something went wrong, I was expecting 120 MHz
    CommsSerial.println("Error: CAN kernel clock frequency is not 120MHz");
    CommsSerial.printf("Actual value: %u\n", can_ker_ck);
    return false;
  }

  const uint32_t prescaler = 3; // or dynamic
  const uint32_t tq_freq = can_ker_ck / prescaler;

  // Check if rate divides evenly without any remainder
  if ((tq_freq % bit_rate) != 0) {
    // Exact baud rate is mathematically impossible with this prescaler/clock
    CommsSerial.println("Error: target bit rate does not divide evenly");
    return false;
  }

  // bit_time_tq unit: tq per bit: (tq / s) / (bit / s)
  uint32_t bit_time_tq = tq_freq / bit_rate;

  // place target sample point at approx. 70%
  uint32_t tq_before_sample = 70 * bit_time_tq / 100;

  // NOTE:
  // bit time = (1 tq (sync time; fixed)) + NominalTimeSeg1 + NominalTimeSeg2

  uint32_t seg_1_tq;
  uint32_t seg_2_tq;

  if (tq_before_sample > 0) {
    /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
    // sample point is after (1 tq + NominalTimeSeg1) so we have to subtract 1tq here
    seg_1_tq = tq_before_sample - 1;
  } else {
    CommsSerial.println("Error: tq_before_sample is zero");
    return false;
  }

  seg_2_tq = bit_time_tq - tq_before_sample;

  if (seg_1_tq > 256 || seg_2_tq > 128 || seg_1_tq < 2 || seg_2_tq < 2) {
    CommsSerial.println("Error: seg_1 or seg_2 calculated values are invalid");
    return false;
  }

  // FDCAN_FilterTypeDef sFilterConfig;

  this->hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  this->hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
  this->hfdcan.Init.AutoRetransmission = ENABLE;
  this->hfdcan.Init.TransmitPause = DISABLE;
  this->hfdcan.Init.ProtocolException = ENABLE;
  this->hfdcan.Init.NominalPrescaler = prescaler;
  this->hfdcan.Init.NominalSyncJumpWidth = 8U < seg_2_tq ? 8U : seg_2_tq;
  this->hfdcan.Init.NominalTimeSeg1 = seg_1_tq;
  this->hfdcan.Init.NominalTimeSeg2 = seg_2_tq;
  this->hfdcan.Init.StdFiltersNbr = 0; // not using any filters on this bus - that way CPU receives all messages (since
                                       // we are just talking to actuators anyway, this is fine)
  this->hfdcan.Init.ExtFiltersNbr = 0;
  this->hfdcan.Init.TxFifoQueueMode =
      FDCAN_TX_FIFO_OPERATION; // other option is queue operation; with fifo messages are sent in the order they are
                               // placed in the fifo, with queue they are sent in order of priority. i think we want
                               // fifo

// platform-dependent options:
// H7 forces the user to determine these values; on the G4 the HAL automatically decides RAM start address and
// buffer/fifo sizes see FDCAN_CalcultateRamBlockAddresses() of stm32g4xx_hal_fdcan.c
#if defined(STM32H7)
  this->hfdcan.Init.MessageRAMOffset =
      (this->hfdcan.Instance == FDCAN1 ? 0 : 1280); // FDCAN1 gets 0-1279, FDCAN2 gets 1280-2559
  this->hfdcan.Init.RxFifo0ElmtsNbr = 8;
  this->hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  this->hfdcan.Init.RxFifo1ElmtsNbr = 0;
  this->hfdcan.Init.RxBuffersNbr = 0;
  this->hfdcan.Init.TxEventsNbr = 0;
  this->hfdcan.Init.TxBuffersNbr = 1;
  this->hfdcan.Init.TxFifoQueueElmtsNbr = 0;
  this->hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
#endif

  // step (1) refers to steps as they are defined by the HAL reference

  // step 1: initialize the FDCAN peripheral

  // pass along the which pins to configure to the MspInit(), which is called during HAL_FDCAN_Init

  if (HAL_FDCAN_Init(&(this->hfdcan)) != HAL_OK) {
    /* Initialization Error */
    CommsSerial.println("Error: HAL_FDCAN_Init failed");
    return false;
  }

  CAN_init_gpio_dynamic(tx_pin, PinMap_CAN_TD);
  CAN_init_gpio_dynamic(rx_pin, PinMap_CAN_RD);

  // step 1 end

  // step 2: configure the FDCAN peripheral
  // filter creation should go around here, but we are not using any (no filtering)

  /* Configure global filter to accept all 11 bit ID frames (and reject remote frames); jhillman todo: confirm the
   * hardware on the bus does not use remote frames */
  if (HAL_FDCAN_ConfigGlobalFilter(&(this->hfdcan), FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
    return false;
  }

  // step 2 end

  // step 3: start the FDCAN
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&(this->hfdcan)) != HAL_OK) {
    /* Start Error */
    return false;
  }
  // step 3 end

  // calculate actual bitrate
  return true;
}

// get number of elements in the receive fifo
size_t CAN::available(void) {
  if (this->hfdcan.Instance == nullptr) {
    return 0;
  }
  return HAL_FDCAN_GetRxFifoFillLevel(&(this->hfdcan), FDCAN_RX_FIFO0);
}

// return 1 if the buffer is empty, 0 otherwise
uint32_t CAN::tx_free_count(void) {
  if (this->hfdcan.Instance == nullptr) {
    return 0;
  }
  return 1 - HAL_FDCAN_IsTxBufferMessagePending(&(this->hfdcan), FDCAN_TX_BUFFER0);
}

bool CAN::flush(uint32_t timeout_ms) {
  if (this->hfdcan.Instance == nullptr) {
    return false;
  }

  uint32_t start_time = millis();

  bool success = false;
  while (millis() - start_time < timeout_ms || timeout_ms == UINT32_MAX) {
    if (HAL_FDCAN_IsTxBufferMessagePending(&(this->hfdcan), FDCAN_TX_BUFFER0) != 1) {
      success = true;
      break;
    }
  }
  return success;
}

int CAN::write(CanMsg const &msg) {
  if (this->hfdcan.Instance == nullptr) {
    return 0;
  }

  FDCAN_TxHeaderTypeDef tx_header = {0};

  /* 1. wait for ongoing transmission to complete: */
  if (this->flush(4U) == false) {
    return 0; // send timed out
  }

  uint32_t id;
  uint32_t id_type;

  if (msg.isStandardId()) {
    id = msg.getStandardId();
    id_type = FDCAN_STANDARD_ID;
  } else if (msg.isExtendedId()) {
    id = msg.getExtendedId();
    id_type = FDCAN_EXTENDED_ID;
  } else // error ? this shouldn't be possible
  {
    return 0;
  }

  if (msg.data_length > 8) {
    // invalid data length
    return 0;
  }

  /* 2. begin new transmission */
  tx_header.Identifier = id;
  tx_header.IdType = id_type;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = msg.data_length;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF; // bit rate switching is off
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // do not store transmission events in the TX event FIFO
  tx_header.MessageMarker = 0;

  HAL_StatusTypeDef status;

#if defined(STM32H7)
  // NOTE: on the H7 this is done in two steps: (1) place the message in the buffer, (2) activate the transmission
  // request place the message in the first buffer
  status = HAL_FDCAN_AddMessageToTxBuffer(&(this->hfdcan), &tx_header, msg.data, FDCAN_TX_BUFFER0);

  if (status != HAL_OK) {
    return 0;
  }

  // enable transmission request
  status = HAL_FDCAN_EnableTxBufferRequest(&(this->hfdcan), FDCAN_TX_BUFFER0);

#elif defined(STM32G4)
  // On the G4 the HAL function HAL_FDCAN_AddMessageToTxFifoQ() both adds the message to the buffer or queue and
  // activates the transmission request
  status = HAL_FDCAN_AddMessageToTxFifoQ(&(this->hfdcan), &tx_header, msg.data);
#else
#error Error
#endif

  return (status == HAL_OK) ? 1 : 0;
}

CanMsg CAN::read(void) {
  if (this->hfdcan.Instance == nullptr) {
    return CanMsg();
  }

  // check that there is room in the rx fifo
  size_t count = this->available();

  if (count == 0) {
    return CanMsg(); // return empty message
  }

  HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&(this->hfdcan), FDCAN_RX_FIFO0, &(this->rx_header), this->rx_data);

  if (status != HAL_OK) {
    return CanMsg();
  }

  uint32_t arduino_msg_id = this->rx_header.Identifier;

  if (this->rx_header.IdType == FDCAN_EXTENDED_ID) {
    arduino_msg_id |=
        CanMsg::CAN_EFF_FLAG; // bit 31 marks it as extended ID format according to the comment in CanMsg.h
  }

  // For classic CAN messages, clamp payload length to 8 bytes maximum (DLC 9-15 indicate 8 bytes in CAN 2.0B)
  uint8_t len = (this->rx_header.DataLength > FDCAN_DLC_BYTES_8) ? 8 : (uint8_t)this->rx_header.DataLength;

  return CanMsg(arduino_msg_id, len, this->rx_data);
}

void CAN::end(void) {
  if (this->hfdcan.Instance == nullptr) {
    return;
  }

  // from the HAL reference: this places the controller back in init mode
  // it should then be legal to eg change baud rate after calling CAN::end() by a successive call to CAN::begin()
  HAL_FDCAN_Stop(&(this->hfdcan));
  return;
}

void CAN::cmd_can_send_test(void) {
  static int count = 0;
  while (!CommsSerial.available()) {
    CanMsg to_send(0x00, 8, (const uint8_t *)"HELLO123");

    if (this->write(to_send) != 1) {
      CommsSerial.println("can.write() failed");
      return;
    }

    uint32_t start_ms = millis();

    while (!this->available()) {
      if (millis() - start_ms > 500) {
        CommsSerial.println("Error: reply timed out");
        return;
      }
    }

    while (this->available()) {
      CanMsg msg = this->read();

      CommsSerial.printf("(%3d) Received message (0x%08X): %.*s\n", count++, msg.id, msg.data_length, msg.data);
    }

    delay(1000);
  }
}

void CAN::cmd_can_receive_test(void) {
  static int count = 0;
  while (!CommsSerial.available()) {
    uint32_t start_ms = millis();

    while (!this->available()) {
      if (millis() - start_ms > 5000) {
        CommsSerial.println("Error: receive timed out");
        return;
      }
    }

    while (this->available()) {
      CanMsg msg = this->read();

      CommsSerial.printf("(%3d) Received message (0x%08X): %.*s\n", count++, msg.id, msg.data_length, msg.data);
    }

    CanMsg to_send(0x00, 8, (const uint8_t *)"REPLY123");

    if (this->write(to_send) != 1) {
      CommsSerial.println("can.write() failed");
      return;
    }

    delay(100);
  }
}