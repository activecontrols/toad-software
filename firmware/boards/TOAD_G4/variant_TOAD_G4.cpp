/*
 *******************************************************************************
 * Copyright (c) 2020, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#include "pins_arduino.h"

// Digital PinName array
const PinName digitalPin[] = {
    PA_0,  // D0/A0
    PA_1,  // D1/A1
    PA_2,  // D2/A2
    PA_3,  // D3/A3
    PA_4,  // D4/A4
    PA_5,  // D5/A5
    PA_6,  // D6/A6
    PA_7,  // D7/A7
    PA_8,  // D8/A8
    PA_9,  // D9/A9
    PA_10, // D10
    PA_11, // D11
    PA_12, // D12
    PA_13, // D13
    PA_14, // D14
    PA_15, // D15
    PB_0,  // D16/A10
    PB_1,  // D17/A11
    PB_2,  // D18/A12
    PB_3,  // D19
    PB_4,  // D20
    PB_5,  // D21
    PB_6,  // D22
    PB_7,  // D23
    PB_8,  // D24
    PB_9,  // D25
    PB_10, // D26
    PB_11, // D27/A13
    PB_12, // D28/A14
    PB_13, // D29/A15
    PB_14, // D30/A16
    PB_15, // D31/A17
    PC_0,  // D32/A18
    PC_1,  // D33/A19
    PC_2,  // D34/A20
    PC_3,  // D35/A21
    PC_4,  // D36/A22
    PC_5,  // D37/A23
    PC_6,  // D38
    PC_7,  // D39
    PC_8,  // D40
    PC_9,  // D41
    PC_10, // D42
    PC_11, // D43
    PC_12, // D44
    PC_13, // D45
    PC_14, // D46
    PC_15, // D47
    PD_2,  // D48
    PF_0,  // D49/A24
    PF_1,  // D50/A25
    PG_10  // D51
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
    0,  // A0,  PA0
    1,  // A1,  PA1
    2,  // A2,  PA2
    3,  // A3,  PA3
    4,  // A4,  PA4
    5,  // A5,  PA5
    6,  // A6,  PA6
    7,  // A7,  PA7
    8,  // A8,  PA8
    9,  // A9,  PA9
    16, // A10, PB0
    17, // A11, PB1
    18, // A12, PB2
    27, // A13, PB11
    28, // A14, PB12
    29, // A15, PB13
    30, // A16, PB14
    31, // A17, PB15
    32, // A18, PC0
    33, // A19, PC1
    34, // A20, PC2
    35, // A21, PC3
    36, // A22, PC4
    37, // A23, PC5
    49, // A24, PF0
    50  // A25, PF1
};

/**
 * @brief  System Clock Configuration
 * @param  None
 * @retval None
 */
WEAK void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
   */
  __HAL_RCC_CRS_CLK_ENABLE();

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configures CRS
   */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /* Peripheral clock enable */
  __HAL_RCC_USB_CLK_ENABLE();

  /* Peripheral interrupt init */
  HAL_NVIC_SetPriority(USB_LP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_IRQn);

  /* Select FDCAN Clock Source */
  PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /* Enable FDCAN Peripheral Clock */
  __HAL_RCC_FDCAN_CLK_ENABLE();
}
