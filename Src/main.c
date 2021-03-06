/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_val.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
ADC_ChannelConfTypeDef adc_sConfig;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define init_v (0)
#define init_f (1)
#define init_dir (1)

int conf_dir(int _dir,int set){
	static int dir=1;
	if(set==1)dir=_dir;

	return dir;
}

int conf_v(int _v,int set){
	static int v=init_v;
	if(set==1)v=_v;

	return v;
}
int conf_f(int _f,int set){
	static int f=init_f;
	if(set==1)f=_f;

	return f;
}


char UART1_Data;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	static int v=init_v;
	static int f=init_f;
	static int dir=init_dir;
     if(UartHandle->Instance==USART1){
          HAL_UART_Receive_IT(&huart1, (uint8_t*) &UART1_Data, 1);
          switch(UART1_Data){
          case 'e':
        	  if(f<50)f+=1;
        	  conf_f(f,1);
        	  break;
          case 'c':
        	  if(f>-50)f-=1;
        	  else v=0;
        	  conf_f(f,1);
        	  break;
          case 'd':
        	  f=init_f;
        	  conf_f(f,1);
        	  break;
          case 'q':
        	  if(v<4000)v+=2;
        	  conf_v(v,1);
        	  break;
          case 'z':
        	  if(v>-4000)v-=2;
        	  else v=0;
        	  conf_v(v,1);
        	  break;
          case 'w':
        	  if(v<4000)v+=10;
        	  conf_v(v,1);
        	  break;
          case 'x':
        	  if(v>-4000)v-=10;
        	  else v=0;
        	  conf_v(v,1);
        	  break;
          case 'a':
        	  v=0;
        	  conf_v(v,1);
        	  break;
          case 'r':
        	  dir=1;
        	  conf_dir(dir,1);
        	  break;
          case 'v':
        	  dir=-1;
        	  conf_dir(dir,1);
        	  break;
          }
     }
}

//HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
//}


void pwm_setvalue(uint16_t value,uint8_t ch)//0<value<4799
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    switch(ch){
    case 1:
    	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    	break;
    case 2:
    	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    	break;
    case 3:
    	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    	break;
    default:
    	break;
    }
}

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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */



  //  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  //  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  //  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);


    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim16);

    HAL_UART_Receive_IT(&huart1, (uint8_t*) &UART1_Data, 1);

    HAL_ADC_Init(&hadc);
    HAL_ADCEx_Calibration_Start(&hadc);
    HAL_ADC_Start_DMA(&hadc,(uint32_t *)aADCxConvertedData,3);

  int neko=0;
  //OC protection selection
  HAL_GPIO_WritePin(OC_SEL_GPIO_Port, OC_SEL_Pin, GPIO_PIN_SET);
  //Enable OC th 250 mV
  HAL_GPIO_WritePin(OC_TH_STBY1_GPIO_Port, OC_TH_STBY1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OC_TH_STBY2_GPIO_Port, OC_TH_STBY2_Pin, GPIO_PIN_SET);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
