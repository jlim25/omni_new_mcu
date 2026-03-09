/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "motorSelection.h"  /* MOTOR_MAX, MOTOR_RECONFIG_CAN_ID */
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* ── RX filter: accept 0x200-0x3FF (RPi_Command_1..6 + RPi_Reconfig) ──────
   * 32-bit mask-mode filter on FIFO0.
   * Layout of 32-bit filter register for standard frames:
   *   [31:21] STID[10:0]  [20:3] unused  [2] IDE  [1] RTR  [0] 0
   * ──────────────────────────────────────────────────────── */
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank           = 0;
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
  /* Accept any standard data frame whose 11-bit ID falls in 0x200-0x3FF.
   * This covers RPi_Command_1..6 (0x201-0x206) and RPi_Reconfig (0x300).
   *
   * Mask strategy: only check ID bits 10:9 = 0b01 (upper two bits = 0x200 range).
   *   FilterIdHigh   = (0x200 << 5) & 0xFFFF = 0x4000  ← reference bits
   *   FilterMaskIdHigh = (0x600 << 5) & 0xFFFF = 0xC000  ← check bits 10:9
   *   FilterMaskIdLow  = 0x0006  ← require IDE=0, RTR=0
   */
  sFilterConfig.FilterIdHigh         = (0x200u << 5u) & 0xFFFFu;
  sFilterConfig.FilterIdLow          = 0x0000u;
  sFilterConfig.FilterMaskIdHigh     = (0x600u << 5u) & 0xFFFFu;
  sFilterConfig.FilterMaskIdLow      = 0x0006u;                    // require IDE=0 (bit2) and RTR=0 (bit1)
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable RX FIFO0 message-pending interrupt */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
      != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
