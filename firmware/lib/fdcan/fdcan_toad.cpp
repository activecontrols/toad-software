#include "stm32h7xx_hal.h"

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


static void Error_Handler(void)
{
  for (;;);
}


// adapted from https://github.com/STMicroelectronics/STM32CubeH7/blob/master/Projects/STM32H743I-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking/Src/stm32h7xx_hal_msp.c
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hfdcan->Instance==FDCAN1)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL; // jhillman: I expect PLL1 Q1 to give 120MHz
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();


    // jhillman TODO: need to fill this out with the correct pins for FDCAN1

    // first enable clock to the GPIOs - I'm guessing arduino already has this enabled, but just in case
    // __HAL_RCC_GPIOX_CLK_ENABLE();
    // etc...


    // GPIO_InitStruct.Pin = GPIO_PIN_X | ...;

    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1; // jhillman TODO: after changing to correct pins, make sure this is the correct alternate function for those pins
    // HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    // HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }

}

namespace CAN1
{

// HAL handle for accessing FDCAN peripheral
static FDCAN_HandleTypeDef hfdcan;

// jhillman adapted from https://github.com/STMicroelectronics/STM32CubeH7/blob/master/Projects/STM32H743I-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking/Src/main.c
// TODO: determine which FDCAN to use (1 or 2) - for now 1 is assumed
void init(void)
{
  // excerpt from the HAL_FDCAN_Init: (since this is for classic CAN, no need to fill out the data bit timing register related fields)

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


  FDCAN_FilterTypeDef sFilterConfig;

  hfdcan.Instance = FDCAN1;
  hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan.Init.AutoRetransmission = ENABLE;
  hfdcan.Init.TransmitPause = DISABLE;
  hfdcan.Init.ProtocolException = ENABLE;
  hfdcan.Init.NominalPrescaler = 3; // jhillman: I expect the peripheral clock to be 120MHz; I divide by 3, therefore kernel clock is 40MHz or 1 tq = 25ns
  hfdcan.Init.NominalSyncJumpWidth = 8;
  hfdcan.Init.NominalTimeSeg1 = 31; /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
  hfdcan.Init.NominalTimeSeg2 = 8; // bit time = (1 tq (sync time; fixed)) + NominalTimeSeg1 + NominalTimeSeg2 = 40 tq = 1us or 1Mbps
  hfdcan.Init.MessageRAMOffset = 0;
  hfdcan.Init.StdFiltersNbr = 0; // not using any filters on this bus - that way CPU receives all messages (since we are just talking to actuators anyway, this is fine)
  hfdcan.Init.ExtFiltersNbr = 0;
  hfdcan.Init.RxFifo0ElmtsNbr = 16;
  hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan.Init.RxFifo1ElmtsNbr = 0;
  hfdcan.Init.RxBuffersNbr = 0;
  hfdcan.Init.TxEventsNbr = 0;
  hfdcan.Init.TxBuffersNbr = 0;
  hfdcan.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; // other option is queue operation; with fifo messages are sent in the order they are placed in the fifo, with queue they are sent in order of priority. i think we want fifo
  hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

  // step 1: initialize the FDCAN peripheral
  if (HAL_FDCAN_Init(&hfdcan) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  // step 1 end

  // step 2: configure the FDCAN peripheral

  // jhillman todo: configure the event fifo

  // filter creation should go around here, but we are not using any (no filtering)

  /* Configure global filter to accept all 11 bit ID frames (and reject remote frames); jhillman todo: confirm the hardware on the bus does not use remote frames */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  // step 2 end

  // step 3: start the FDCAN
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  // step 3 end

  // if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  // {
  //   /* Notification Error */
  //   Error_Handler();
  // }

  return;  
}


// get number of elements in the receive fifo
uint32_t rcv_count(void)
{
  return HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0);
}

// get number free positions in the transmit fifo
uint32_t tx_free_count(void)
{
  return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan);
}

}// namespace CAN1