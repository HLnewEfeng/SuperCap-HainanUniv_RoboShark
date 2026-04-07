/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t adc_ready_flag=0;

volatile uint16_t adc_buf[5]={0};
float Vcap=0, Vbat=0;
float Icap=0, Ibat=0;
float Pbat=0,Pcap=0,Pref=15;

int initialstart=1;//
int capemerged = 0;

int Duty = 0;

/*
PID Data Structure
*/
PI_Controller pi_pbat =
{
  .kp      = 0.05f,
  .ki      = 12.0f,
  .ts      = 0.01f,

  .integral = 0.0f,
  .out_min  = Icap_chg_max_i,   // ??????
  .out_max  = Icap_dis_max_i    // ??????
};
PI_Controller pi_pbat_SS =
{
  .kp      = 0.05f,//0.05
  .ki      = 12.0f,//12
  .ts      = 0.01f,//0.0005

  .integral = 0.0f,
  .out_min  = Icap_CC,   // ??????
  .out_max  = 0    // ??????
};
PI_Controller pi_icap =
{
  .kp      = 0.18f,
  .ki      = 600.0f,//-------------------------------
  .ts      = 0.00002f,

  .integral = 0.0f,
  .out_min  = D_MIN_i,
  .out_max  = D_MAX_i
};

enum CapProtection
{Default=0,BatOverCurrent,CapDischargeOC,CapChargeOC,CapOverVoltage,ADCOffline,Normal,BatOVP}
errName=6;
enum CHARGESTATUS
{SoftStart=1,Norm=2}
ChargeState=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
  capemerged=0;
  // 1. 先启动FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  // 2. 再配置过滤器和中断
  FDCAN_Filter_Init();

  HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

  Protection();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim1);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
ADC-Converting
*/
void ADC_ConvertToPhysical(){
  Vbat=( 0.00848 * adc_buf[0] +  0.62);
  Vcap=( 0.00844 * adc_buf[1] +  0.56);
  Ibat=( 0.00402 * adc_buf[2] -  0.01);
  Icap=( 0.01957 * adc_buf[3] - 38.67);
  //Temp=adc_buf[4];
}

/*
PowerCalculating
*/
void Physical_Calc(){
  Pbat=Ibat*Vbat;
  Pcap=Icap*Vcap;
}

/*
Limit
*/
float LIMIT(float p,float min,float max){
  if (p < min) return min;
  if (p > max) return max;
  return p;
}

/*
PI_RUN
based on a structed PI
*/
float PI_Run(PI_Controller *pi,float ref,float fdb)
{
  float err = ref - fdb;

  /* ?? */
  pi->integral += pi->ki * err * pi->ts;

  /* anti-windup:???? */
  if (pi->integral > pi->out_max)
    pi->integral = pi->out_max;
  else if (pi->integral < pi->out_min)
    pi->integral = pi->out_min;

  /* ?? */
  float out = pi->kp * err + pi->integral;

  /* ???? */
  if (out > pi->out_max)
    out = pi->out_max;
  else if (out < pi->out_min)
    out = pi->out_min;

  return out;
}

/*
PI_Control
*/
float Pbat_Control(float Pref,float Pbat,float Vcap)
{
  if (Vcap < Vcap_CC)//SoftStart
  {
    StopRunning(0);
    return PI_Run(&pi_pbat_SS, Pbat, Pref);
  }
  StopRunning(0);
  return PI_Run(&pi_pbat, Pbat, Pref);
}
float Icap_Control(float Icap_ref,float Icap)
{
  return PI_Run(&pi_icap, Icap, Icap_ref);
}

/*
StopRunning
*/
void StopRunning(int errCode){
  if(errCode){
    if(initialstart){
      TIM1->BDTR &= ~TIM_BDTR_MOE;
      //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
      capemerged=0;
      errName=errCode;
      return;
    }
  }
  else if(Vcap>Vcap_CC){
    ChargeState=2;
    return;
  }
  else {
    ChargeState=1;
    return;
  }
}

/*
Protection_Detection
*/
int ProtectionCnt=0;
void Protection(){//
  if(Ibat > Ibat_max){
    if(ProtectionCnt == 3) StopRunning(1);
    else ProtectionCnt+=1;
  }
  else if(Icap >  Icap_max){
    if(ProtectionCnt == 3) StopRunning(2);
    else ProtectionCnt+=1;
  }
  else if(Icap <  Icap_min ) {
    if(ProtectionCnt == 3) StopRunning(3);
    else ProtectionCnt+=1;
  }
  else if(Vcap > Vcap_max) {
    //if(ProtectionCnt == 3)
    StopRunning(4);
    //else ProtectionCnt+=1;
  }
  else if(Vbat > Vbat_max) {
    //if(ProtectionCnt == 3)
    StopRunning(7);
    //else ProtectionCnt+=1;
  }
  else ProtectionCnt = 0;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
