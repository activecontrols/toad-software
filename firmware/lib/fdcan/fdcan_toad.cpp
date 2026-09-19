#include "fdcan_toad.h"

#include "Arduino.h"

#include "stm32h7xx_hal.h"
#include "variant_TOAD_H7.h"
#include "ec_pins.h"

#include "CommsSerial.h"

using namespace arduino;

/*
jhillman notes:
So far the goal is to get one FDCAN interface working in normal CAN mode for interfacing with the actuators

- time quanta (tq): derived from the kernel clock, the discrete unit of time the CAN core operates on
- minimum time quanta (mtq): one period of the kernel clock (fdcan_tq_ck) [RM p2621]
- two clock domains, APB and kernel (peripheral) clock
  - make sure to run kernel clock at a frequency no greater than APB
  - CAN core and calibration unit operates using kernel clock, the rest of the module operates on APB clock [RM p2614]
- there's configurable ram for holding id filters and tx/rx fifos or buffers [RM p2627]
  - supported filters: range, dedicated ID, or bit mask filter [AN5348 p12]
  - each filter can be configured for acceptance or rejection
  - buffer can only store up to 1 element [AN5348 p16], fifo can store many [AN5348 p14]
  - make sure to align ram elements to words (32 bits) - HAL probably does this automatically [RM p2626]
  - if filters are disabled, all messages are accepted (we likely want this one) [RM p2629]
- 11 and 29 bit identifiers supported [RM p2617]
- tx event fifo (optional, probably don't use) [AN5348 p17]
  - tells CPU transmission order, time of transmission
- tx buffers, tx queues, tx FIFO [AN5348 p18] (see table 7 for comparison)
  - cannot use both queues and FIFO in the same application
  - buffer: stores a single message until it is sent
  - fifo: transmission order depends on the order in which messages are placed in the fifo, not their priorities [AN5348 p20]
  - queue: transmission order depends on priority of the message, not the order they are placed in the queue [AN5348 p20]
- delay compensation [AN5348 p26] (optional feature, not needed for normal CAN operation)
  - loop delay is the delay inherent in the connected between and in the transceiver such that there is a delay between an edge being transmitted on FDCAN_TX and being received on FDCAN_RX [RM p2620]
  - typically the loop delay between CAN controller and transceiver places an upper limit on the bitrate [AN5348 p28]
    - using delay compensation feature inserts a second sample point (SSP) used instead of the usual sample point to get around this upper limit by allowing the sent bit to be detected after the CAN controller sends the next bit
  - delay compensation is disabled during arbitration phase [AN5348 p27]
  - the CAN controller measures the delay compensation during the arbitration phase and uses this to define the position of the SSP during the data phase [AN5348 p27] [RM p2621]
- bit timing [RM p2638]
  - SYNC_SEG (synchronization segment) is fixed to 1 tq; a bit change is expected in this segment
  - bit segment 1 (BS1) controls when the sample point occurs; (sample occurs at (SYNC_SEG + BS1) tq)
  - bit segment 2 (BS2) controls when the transmit point occurs w.r.t. the sample point
  - baudrate = 1 / (bit time) = 1 / (SYNC_SEG + BS1 + BS2) for normal CAN mode [RM p2639]
  - !!! BS1 and BS2 are automatically adjusted by the FDCAN after initialization to account for drift; the synchronization jump width defines the maximum amount by which these may change; the RM says this is limited to four or less, online sources indicate otherwise (limit is much higher) [RM p2639]
- FDCAN1 has an application watchdog that must either be frequently served or can be disabled [RM 2625]

- FDCAN_InitTypeDef [UM p495] for intialization
- FDCAN_FilterTypeDef [UM p497] for defining id filters
- FDCAN_TxHeaderTypeDef [UM p498]
- FDCAN_RxHeaderTypeDef [UM p499]
- FDCAN_HpMsgStatusTypeDef [UM p501]
- FDCAN_ProtocolStatusTypeDef [UM p501]
- FDCAN_ErrorCountersTypeDef [UM p502]
- FDCAN_MsgRamAddressTypeDef [UM p506]
- __FDCAN_HandleTypeDef (FDCAN_HandleTypeDef) [UM p507]

references:
AN5348 Rev 6 (Introduction to FDCAN peripherals for STM32 MCUs)
RM0399 (RM) Rev 4 (STM32H745/755 and STM32H747/757 advanced Arm-based 32-bit MCUs)
UM2217 (UM) Rev 6 (Description of STM32H7 HAL and low-layer drivers)


for actuator CAN: likely configuration is to store all incoming messages in rx FIFO 0, no filtering
*/

static CAN* can1 = nullptr;
static CAN* can2 = nullptr;

struct CAN::HAL
{
  FDCAN_ErrorCountersTypeDef error_counts = {0};
  FDCAN_HandleTypeDef hfdcan = {0};
  FDCAN_RxHeaderTypeDef rx_header = {0};
};


/* enable GPIO clock and configure pin (must pass a bit mask for which pin(s) to configure on the specific port) */
static void CAN_init_gpio_dynamic(uint32_t pin, const PinMap pin_map[])
{

  PinName pin_name = digitalPinToPinName(pin);

  // jhillman: I confirmed this enables GPIO clock in the RCC, also sets GPIO speed to very high
  pin_function(pin_name, pinmap_function(pin_name, pin_map));
}


// adapted from https://github.com/STMicroelectronics/STM32CubeH7/blob/master/Projects/STM32H743I-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking/Src/stm32h7xx_hal_msp.c
// this is called by STM32 HAL during HAL_FDCAN_Init()


extern "C" 
{

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  static bool has_initialized_clock = false;

  if (!has_initialized_clock)
  {
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL; // jhillman: I expect PLL1 Q1 to give 120MHz
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }


    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    has_initialized_clock = true;
  }
 

  CAN* target = (hfdcan->Instance == FDCAN1) ? can1 : can2;
  if (target != nullptr)
  {
    CAN_init_gpio_dynamic(target->rx_pin, PinMap_CAN_RD);
    CAN_init_gpio_dynamic(target->tx_pin, PinMap_CAN_TD);
  }


  IRQn_Type irqn = FDCAN1_IT0_IRQn; // each FDCAN has two interrupt lines but we can just use line 0

  if (hfdcan->Instance == FDCAN2) 
  {
    irqn = FDCAN2_IT0_IRQn;
  }
  /* FDCAN1 interrupt Init */
  HAL_NVIC_SetPriority(irqn, 0, 0);
  HAL_NVIC_EnableIRQ(irqn);
}


void fdcan_error_cbk(FDCAN_HandleTypeDef* hfdcan)
{
  CAN* target = nullptr;

  if (hfdcan->Instance == FDCAN1)
  {
    target = can1;
  }
  else if (hfdcan->Instance == FDCAN2)
  {
    target = can2;
  }

  if (target == nullptr)
  {
    return;
  }

  target->error_update_from_isr();

  return;
}

void fdcan_error_status_cbk(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
  fdcan_error_cbk(hfdcan);
}


void FDCAN1_IT0_IRQHandler(void)
{
  if (can1 != nullptr)
  {
    HAL_FDCAN_IRQHandler(&(can1->hal->hfdcan));
  }
}


void FDCAN2_IT0_IRQHandler(void)
{
  if (can2 != nullptr)
  {
    HAL_FDCAN_IRQHandler(&(can2->hal->hfdcan));
  }
}

} // extern "C"



CAN::~CAN() = default;

CAN::CAN(uint32_t _tx_pin, uint32_t _rx_pin)
{
  FDCAN_GlobalTypeDef* inst_1 = static_cast<FDCAN_GlobalTypeDef*>(pinmap_find_peripheral(digitalPinToPinName(_tx_pin), PinMap_CAN_TD));
  FDCAN_GlobalTypeDef* inst_2 = static_cast<FDCAN_GlobalTypeDef*>(pinmap_find_peripheral(digitalPinToPinName(_rx_pin), PinMap_CAN_RD));

  this->hal = std::make_unique<CAN::HAL>();
  
  if ((inst_1 != inst_2 ) || inst_1 == NP)
  {
    Error_Handler();
  }
 

  this->hal->hfdcan = {0};
  this->hal->hfdcan.Instance = inst_1;

  this->tx_pin = _tx_pin;
  this->rx_pin = _rx_pin;
}


// jhillman adapted from https://github.com/STMicroelectronics/STM32CubeH7/blob/master/Projects/STM32H743I-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking/Src/main.c
float CAN::begin(uint32_t bit_rate)
{
  // excerpt from the HAL_FDCAN_Init: (since this is for classic CAN, no need to fill out the data bit timing register related fields since those are only used when bit rate switching is enabled)

  // /* Set the nominal bit timing register */
  // hfdcan->Instance->NBTP = ((((uint32_t)hfdcan->Init.NominalSyncJumpWidth - 1U) << FDCAN_NBTP_NSJW_Pos) |
  //                           (((uint32_t)hfdcan->Init.NominalTimeSeg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos)    |
  //                           (((uint32_t)hfdcan->Init.NominalTimeSeg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos)    |
  //                           (((uint32_t)hfdcan->Init.NominalPrescaler - 1U) << FDCAN_NBTP_NBRP_Pos));

  // /* If FD operation with BRS is selected, set the data bit timing register */
  // if (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS)
  // {
  //   hfdcan->Instance->DBTP = ((((uint32_t)hfdcan->Init.DataSyncJumpWidth - 1U) << FDCAN_DBTP_DSJW_Pos)  |
  //                             (((uint32_t)hfdcan->Init.DataTimeSeg1 - 1U) << FDCAN_DBTP_DTSEG1_Pos)     |
  //                             (((uint32_t)hfdcan->Init.DataTimeSeg2 - 1U) << FDCAN_DBTP_DTSEG2_Pos)     |
  //                             (((uint32_t)hfdcan->Init.DataPrescaler - 1U) << FDCAN_DBTP_DBRP_Pos));
 
  if (this->hal->hfdcan.Instance == FDCAN1)
  {
    can1 = this;
  }
  else if (this->hal->hfdcan.Instance == FDCAN2)
  {
    can2 = this;
  }
  else
  {
    CommsSerial.println("Error: only FDCAN1 or 2 are supported\n");
    Error_Handler(); 
  }

  if (bit_rate == 0U || bit_rate > 1'000'000U)
  {
    // not allowed for normal CAN operation
    Error_Handler();
  }
  // TODO: audit this

  const uint32_t can_ker_ck = 120'000'000; // peripheral clock is 120 MHz - this is hardcoded
  const uint32_t prescaler = 3;            // or dynamic
  const uint32_t tq_freq = can_ker_ck / prescaler;

  // Check if rate divides evenly without any remainder
  if ((tq_freq % bit_rate) != 0) {
    // Exact baud rate is mathematically impossible with this prescaler/clock

    CommsSerial.println("Error: CAN target bit rate not possible with this clock configuration.\n");
    Error_Handler();
  }

  // bit_time_tq unit: tq per bit: (tq / s) / (bit / s)
  uint32_t bit_time_tq = tq_freq / bit_rate;

  // place target sample point at approx. 70%
  uint32_t tq_before_sample = 70 * bit_time_tq / 100;

  // NOTE:
  // bit time = (1 tq (sync time; fixed)) + NominalTimeSeg1 + NominalTimeSeg2

  uint32_t seg_1_tq;
  uint32_t seg_2_tq;

  if (tq_before_sample > 0)
  {
    /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
    // sample point is after (1 tq + NominalTimeSeg1) so we have to subtract 1tq here
    seg_1_tq = tq_before_sample - 1;
  }
  else
  {
    // shouldn't happen
    Error_Handler();
  }

  seg_2_tq = bit_time_tq - tq_before_sample;

  if (seg_1_tq > 256 || seg_2_tq > 128 || seg_1_tq < 2 || seg_2_tq < 2)
  {
    CommsSerial.println("Error: Bit timing segments exceed hardware register limits.");
    Error_Handler();
  }

  // FDCAN_FilterTypeDef sFilterConfig;

  this->hal->hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  this->hal->hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
  this->hal->hfdcan.Init.AutoRetransmission = ENABLE;
  this->hal->hfdcan.Init.TransmitPause = DISABLE;
  this->hal->hfdcan.Init.ProtocolException = ENABLE;
  this->hal->hfdcan.Init.NominalPrescaler = prescaler;
  this->hal->hfdcan.Init.NominalSyncJumpWidth = 8U < seg_2_tq ? 8U : seg_2_tq;
  this->hal->hfdcan.Init.NominalTimeSeg1 = seg_1_tq;
  this->hal->hfdcan.Init.NominalTimeSeg2 = seg_2_tq;
  this->hal->hfdcan.Init.MessageRAMOffset = (this->hal->hfdcan.Instance == FDCAN1 ? 0 : 1280); // FDCAN1 gets 0-1279, FDCAN2 gets 1280-2559
  this->hal->hfdcan.Init.StdFiltersNbr = 0; // not using any filters on this bus - that way CPU receives all messages (since we are just talking to actuators anyway, this is fine)
  this->hal->hfdcan.Init.ExtFiltersNbr = 0;
  this->hal->hfdcan.Init.RxFifo0ElmtsNbr = 8;
  this->hal->hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  this->hal->hfdcan.Init.RxFifo1ElmtsNbr = 0;
  this->hal->hfdcan.Init.RxBuffersNbr = 0;
  this->hal->hfdcan.Init.TxEventsNbr = 0;
  this->hal->hfdcan.Init.TxBuffersNbr = 1;
  this->hal->hfdcan.Init.TxFifoQueueElmtsNbr = 0;
  this->hal->hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; // other option is queue operation; with fifo messages are sent in the order they are placed in the fifo, with queue they are sent in order of priority. i think we want fifo
  this->hal->hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  this->hal->hfdcan.ErrorCallback = fdcan_error_cbk;
  this->hal->hfdcan.ErrorStatusCallback = fdcan_error_status_cbk;


  // step (1) refers to steps as they are defined by the HAL reference

  // step 1: initialize the FDCAN peripheral

  // pass along the which pins to configure to the MspInit(), which is called during HAL_FDCAN_Init

  if (HAL_FDCAN_Init(&(this->hal->hfdcan)) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  // step 1 end

  // step 2: configure the FDCAN peripheral
  // filter creation should go around here, but we are not using any (no filtering)

  /* Configure global filter to accept all 11 bit ID frames (and reject remote frames); jhillman todo: confirm the hardware on the bus does not use remote frames */
  if (HAL_FDCAN_ConfigGlobalFilter(&(this->hal->hfdcan), FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    CommsSerial.println("HAL_FDCAN_ConfigGlobalFilter() failed");

    Error_Handler();
  }

  constexpr uint32_t it_list = FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING;

  if (HAL_FDCAN_ConfigInterruptLines(&(this->hal->hfdcan), it_list, FDCAN_INTERRUPT_LINE0) != HAL_OK)
  {
    CommsSerial.println("HAL_FDCAN_ConfigInterruptLines() failed");
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(
      &(this->hal->hfdcan),
      it_list, // when either the tx error counter or the rx error counter overflows, this interrupt is triggered (RM p.2685)
      0x00
    ) != HAL_OK)
  {
    CommsSerial.println("HAL_FDCAN_ActivateNotification() failed");
    Error_Handler();
  }

  // step 2 end

  // step 3: start the FDCAN
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&(this->hal->hfdcan)) != HAL_OK)
  {
    CommsSerial.println("HAL_FDCAN_Start() failed");
    /* Start Error */
    Error_Handler();
  }
  // step 3 end

  // if (HAL_FDCAN_ActivateNotification(&(this->hal->hfdcan), FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
  // {
  //   /* Notification Error */
  //   Error_Handler();
  // }

  // calculate actual bitrate
  return ((float)tq_freq / (1 + seg_1_tq + seg_2_tq));
}


// for compliance with the arduino api spec
// this function always just returns true because it will enter an error handler routine in the event of a failure
bool CAN::begin(CanBitRate bit_rate)
{
  this->begin((uint32_t)bit_rate);

  return true;
}

// get number of elements in the receive fifo
size_t CAN::available(void)
{
  return HAL_FDCAN_GetRxFifoFillLevel(&(this->hal->hfdcan), FDCAN_RX_FIFO0);
}


// return 1 if the buffer is empty, 0 otherwise
uint32_t CAN::tx_free_count(void)
{
  return 1 - HAL_FDCAN_IsTxBufferMessagePending(&(this->hal->hfdcan), FDCAN_TX_BUFFER0);
}


bool CAN::flush(uint32_t timeout_ms)
{
  uint32_t start_time = millis();

  bool success = false;
  while (millis() - start_time < timeout_ms || timeout_ms == UINT32_MAX)
  {
    if (HAL_FDCAN_IsTxBufferMessagePending(&(this->hal->hfdcan), FDCAN_TX_BUFFER0) != 1)
    {
      success = true;
      break;
    }
  }
  return success;
}

int CAN::write(CanMsg const & msg)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};

  /* 1. wait for ongoing transmission to complete: */
  if (this->flush(4U) == false)
  {
    return 0; // send timed out
  }

  uint32_t id;
  uint32_t id_type;

  if (msg.isStandardId())
  {
    id = msg.getStandardId();
    id_type = FDCAN_STANDARD_ID;
  }
  else if (msg.isExtendedId())
  {
    id = msg.getExtendedId();
    id_type = FDCAN_EXTENDED_ID;
  }
  else // error ? this shouldn't be possible
  {
    return 0;
  }

  if (msg.data_length > 8)
  {
    // invalid data length
    return 0;
  }

  /* 2. begin new transmission */
  tx_header.Identifier = id;
  tx_header.IdType = id_type;
  tx_header.DataLength = msg.data_length;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF; // bit rate switching is off
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // events will go in the tx fifo, for feedback so we know whether transmission was successful or not
  tx_header.MessageMarker = 0;

  // place the message in the first buffer
  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxBuffer(&(this->hal->hfdcan), &tx_header, msg.data, FDCAN_TX_BUFFER0);

  if (status != HAL_OK)
  {
    return false;
  }

  status = HAL_FDCAN_EnableTxBufferRequest(&(this->hal->hfdcan), FDCAN_TX_BUFFER0);

  return (status == HAL_OK) ? 1 : 0;
}


CanMsg CAN::read(void)
{
  // check that there is room in the rx fifo
  size_t count = this->available();

  if (count == 0)
  {
    return CanMsg(); // return empty message
  }
  
  HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&(this->hal->hfdcan), FDCAN_RX_FIFO0, &(this->hal->rx_header), this->rx_data);

  if (status != HAL_OK)
  {
    return CanMsg();
  }

  uint32_t arduino_msg_id = this->hal->rx_header.Identifier;

  if (this->hal->rx_header.IdType == FDCAN_EXTENDED_ID)
  {
    arduino_msg_id |= CanMsg::CAN_EFF_FLAG; // bit 31 marks it as extended ID format according to the comment in CanMsg.h
  }

  // for standard CAN messages: DLC <= 8 means data_length = DLC
  if (this->hal->rx_header.DataLength > FDCAN_DLC_BYTES_8)
  {
    return CanMsg(); // return empty message because an error occured (invalid DLC for standard CAN)
  }

  return CanMsg(arduino_msg_id, this->hal->rx_header.DataLength, this->rx_data);
}

void CAN::end(void)
{
  // from the HAL reference: this places the controller back in init mode
  // it should then be legal to eg change baud rate after calling CAN::end() by a successive call to CAN::begin()
  HAL_FDCAN_Stop(&(this->hal->hfdcan));
  return;
}

void CAN::set_error_cbk(CAN_error_cbk_t error_cbk)
{
  // this should be an atomic operation, so there is no need to guard
  this->error_cbk = error_cbk;
  return;
}

// note: must only be called from an interrupt context, from a single isr
void CAN::error_update_from_isr(void)
{
  FDCAN_ErrorCountersTypeDef new_error_counts = {0};

  HAL_FDCAN_GetErrorCounters(&(this->hal->hfdcan), &new_error_counts);

  if (this->error_cbk == nullptr)
  {
    memcpy(&(this->hal->error_counts), &new_error_counts, sizeof(new_error_counts));
    return;
  }

  if (new_error_counts.RxErrorCnt != this->hal->error_counts.RxErrorCnt)
  {
    this->error_cbk(CAN::error_type_t::RX_ERROR, new_error_counts.RxErrorCnt);
  }

  if (new_error_counts.RxErrorPassive != this->hal->error_counts.RxErrorPassive)
  {
    this->error_cbk(CAN::error_type_t::RX_ERROR_PASSIVE, new_error_counts.RxErrorPassive);
  }

  if (new_error_counts.TxErrorCnt != this->hal->error_counts.TxErrorCnt)
  {
    this->error_cbk(CAN::error_type_t::TX_ERROR, new_error_counts.TxErrorCnt);
  }

  memcpy(&(this->hal->error_counts), &new_error_counts, sizeof(new_error_counts));
  return;
}