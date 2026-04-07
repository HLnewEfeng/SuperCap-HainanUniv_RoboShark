/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
CAN_ReceiveDataTypeDef sCAN_RX_data;
CAN_TransmitDataTypeDef sCAN_TX_data;
uint8_t CAN_ReceiveDataRefresh_Flag = 1;
uint8_t CAN_WDG_Count = 0;

FDCAN_RxHeaderTypeDef sFDCAN1_RxHeader;
FDCAN_TxHeaderTypeDef sFDCAN1_TxHeader;
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 24;
  hfdcan1.Init.NominalTimeSeg2 = 7;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * 过滤器初始化函数
 */
void FDCAN_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig; // 设为局部变量即可

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // 标准CANID
    sFilterConfig.FilterIndex  = 0;                       // 过滤器序列0
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;       // 双ID过滤模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 存入 Rx FIFO0
    sFilterConfig.FilterID1    = CAN_ID_BUS_TO_SUPERCAP;  // 0x221
    sFilterConfig.FilterID2    = CAN_TEST_ID;             // 0x222

    // 配置过滤器
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 全局过滤器配置：拒绝未匹配的帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // ★ 关键修复：激活 RX FIFO0 新消息中断，否则不会进回调！
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * fdcan接收回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t CAN_RX_Buff[8];

    if (hfdcan->Instance == FDCAN1)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &sFDCAN1_RxHeader, CAN_RX_Buff) == HAL_OK)
        {
            // CAN看门狗计数清零
            CAN_WDG_Count = 0;

            if (CAN_ReceiveDataRefresh_Flag)
            {
                sCAN_RX_data.Enable      = CAN_RX_Buff[0];
                sCAN_RX_data.Charge      = CAN_RX_Buff[1];
                sCAN_RX_data.PowerLimit  = CAN_RX_Buff[2];
                sCAN_RX_data.ChargePower = CAN_RX_Buff[3];

                // 清除标志位，等待主循环处理后再置1
                CAN_ReceiveDataRefresh_Flag = 0;
            }
        }
        // HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}

/**
 * FDCAN发送函数
 */
void Can_SendMess(CAN_TransmitDataTypeDef *TX_temp)
{
    static uint8_t FirstSend = 1; // 设为局部静态变量更内聚

    if (FirstSend) {
        sFDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        sFDCAN1_TxHeader.FDFormat      = FDCAN_CLASSIC_CAN; // 经典CAN，满足8字节限制
        sFDCAN1_TxHeader.Identifier    = CAN_ID_SUPERCAP_TO_BUS; // 0x220
        sFDCAN1_TxHeader.IdType        = FDCAN_STANDARD_ID;
        sFDCAN1_TxHeader.DataLength    = FDCAN_DLC_BYTES_8;
        sFDCAN1_TxHeader.TxFrameType   = FDCAN_DATA_FRAME;

        // ★ 补齐必要的 FDCAN 特有设置
        // sFDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        // sFDCAN1_TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        // sFDCAN1_TxHeader.MessageMarker       = 0;

        FirstSend                      = 0;
    }

    uint8_t CAN_TX_BUFF[8] = {0};
    CAN_TX_BUFF[0]         = 1;//(uint8_t)TX_temp->SuperCapReady;
    CAN_TX_BUFF[1]         = 1;//(uint8_t)TX_temp->SuperCapState;
    CAN_TX_BUFF[2]         = TX_temp->SuperCapEnergy;
    CAN_TX_BUFF[3]         = TX_temp->ChassisPower;
    CAN_TX_BUFF[4]         = TX_temp->VoltageBat;
    CAN_TX_BUFF[5]         = TX_temp->DebugOut_BatPower;
    CAN_TX_BUFF[6]         = (uint8_t)TX_temp->DebugOut_SuperCapPower;
    // CAN_TX_BUFF[7] 默认是 0

    // if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
    // {
    //     return;// HAL_BUSY;
    // }
    // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sFDCAN1_TxHeader, CAN_TX_BUFF);

    // 检查FIFO是否有空间
    // if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sFDCAN1_TxHeader, CAN_TX_BUFF);
    //}
}
/* USER CODE END 1 */
