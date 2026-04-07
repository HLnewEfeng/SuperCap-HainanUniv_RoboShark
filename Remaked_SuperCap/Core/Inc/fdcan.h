/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
// 宏定义
#define CAN_ID_SUPERCAP_TO_BUS  0x220
#define CAN_ID_BUS_TO_SUPERCAP  0x221
#define CAN_TEST_ID             0x222

// 超级电容状态枚举
typedef enum {
  SUPERCAP_UNREADY = 0,
  SUPERCAP_READY = 1
} SuperCapReadyTypeDef;

typedef enum {
  E_SUPERCAP_STATE_NORMAL = 0,
  E_SUPERCAP_STATE_ERROR = 1
} SuperCapStateTypeDef;

// 接收结构体 (电控发来，13字节拆分)
typedef struct {
  uint8_t Charge ;              // 超级电容充电。1充电，0放电
  uint8_t Enable ;              // 超级电容给使能。1使能，0失能
  uint8_t PowerLimit;           // 裁判系统功率限制
  uint8_t ChargePower;          // 超级电容充电功率
  uint8_t Dead;                 // 机器人死亡状态
  float PowerLimitAfterOffset;  // 本地计算的补偿限制
  float ChargePowerAfterOffset; // 本地计算的补偿功率
} CAN_ReceiveDataTypeDef;

// 发送结构体 (发给电控，7字节)
typedef struct {
  uint8_t SuperCapEnergy;       // 超级电容可用能量：0-100%
  uint8_t ChassisPower;         // 底盘实时功率：0-255
  SuperCapReadyTypeDef SuperCapReady; // 超级电容【可用标志】
  SuperCapStateTypeDef SuperCapState; // 超级电容【状态标志】
  uint8_t VoltageBat;           // 电池电压*10
  uint8_t DebugOut_BatPower;
  int8_t DebugOut_SuperCapPower;
} CAN_TransmitDataTypeDef;

// 暴露给外部的全局变量和函数
extern CAN_ReceiveDataTypeDef sCAN_RX_data;
extern CAN_TransmitDataTypeDef sCAN_TX_data;
extern uint8_t CAN_ReceiveDataRefresh_Flag;
extern uint8_t CAN_WDG_Count;

void FDCAN_Filter_Init(void);
void Can_SendMess(CAN_TransmitDataTypeDef *TX_temp);
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

