/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define Icap_chg_max_i -2 //-20
#define Icap_dis_max_i 3 //30
#define Icap_min -6 //-20
#define Icap_max 9 //30
#define Icap_CC -2 //-20
#define Vcap_CC 7//缓启动阈值
#define D_MIN_i 5
#define D_MAX_i 389//389
#define D_min 5
#define D_max 389
#define Ibat_max 10 //10
#define Vcap_max 26
#define Vbat_max 28

typedef struct
{
  float kp;
  float ki;
  float ts;

  float integral;
  float out_min;
  float out_max;
} PI_Controller;

void ADC_ConvertToPhysical();
void Physical_Calc();
float LIMIT(float p,float min,float max);
float PI_Run(PI_Controller *pi,float ref,float fdb);
float Pbat_Control(float Pref,float Pbat,float Vcap);
float Icap_Control(float Icap_ref,float Icap);
void StopRunning(int errCode);
void Protection();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
