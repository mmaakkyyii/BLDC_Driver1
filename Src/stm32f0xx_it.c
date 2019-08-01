/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "my_val.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char debug_data[20]={};
void Debug(char* data,int size){
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)data,(uint16_t)size);
}


//sin(deg)*100
const int sin_val[90]={0 ,2 ,3 ,5 ,7 ,9 ,10 ,12 ,14 ,16 ,17 ,19 ,21 ,22 ,24 ,26 ,28 ,29 ,31 ,33 ,34 ,36 ,37 ,39 ,41 ,42 ,44 ,45 ,47 ,48 ,50 ,52 ,53 ,54 ,56 ,57 ,59 ,60 ,62 ,63 ,64 ,66 ,67 ,68 ,69 ,71 ,72 ,73 ,74 ,75 ,77 ,78 ,79 ,80 ,81 ,82 ,83 ,84 ,85 ,86 ,87 ,87 ,88 ,89 ,90 ,91 ,91 ,92 ,93 ,93 ,94 ,95 ,95 ,96 ,96 ,97 ,97 ,97 ,98 ,98 ,98 ,99 ,99 ,99 ,99 ,100 ,100 ,100 ,100 ,100 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int my_sin(int deg){
	if(deg>=0){
		while(deg>360){
			deg=-360;
		}
	}else{
		while(deg<0){
			deg=deg+360;
		}
	}
	if(deg<0)return 0;
	else if(deg<90)return sin_val[deg];
	else if(deg==90)return 1000;
	else if(deg<=180)return sin_val[180-deg];
	else if(deg<270)return -sin_val[deg-180];
	else if(deg==270)return -1000;
	else if(deg<=360)return -sin_val[360-deg];
	else return 0;

}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim14;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
//	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
//	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
	//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);

    HAL_ADC_Start_DMA(&hadc,(uint32_t *)aADCxConvertedData,3);
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

	static const int16_t ENC_ZERO=0x7FFF;

	int16_t pulse=TIM2->CNT-ENC_ZERO;
	TIM2->CNT=ENC_ZERO;

	static int v=1000;
	v=conf_v(0,-1);

	static float t=0;
//	v++;
//	if(v>4799/2)v=0;
/*
	int f=1;
	int sin_v=0;
	int a=1000;//max 4799

	for(int i=1;i<=3;i++){
    	//duty=V/250.0f*(100+100*sin((t*2*3.14*f)+(2-i)*(2.0f/3.0f*3.14f) ));
    	sin_v=my_sin( ( (t*360*f)+(2-i)*(120) ));
    	int duty=a*(100+sin_v)/100.0f;
    	pwm_setvalue((uint16_t)duty,i);
    }
    t+=0.001;
    if(t*f>1){
    	t=0;
    }
*/
//*
  int h3=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
  int h1=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
  int h2=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

  int hall_state=h1+h2*2+h3*4;

  if(v>0){
	  switch(hall_state){
	  case 0b101:
			pwm_setvalue(0,1);
			pwm_setvalue(v,2);
			pwm_setvalue(0,3);
			break;
	  case 0b100:
			pwm_setvalue(0,1);
			pwm_setvalue(0,2);
			pwm_setvalue(v,3);
			break;
	  case 0b110:

			pwm_setvalue(0,1);
			pwm_setvalue(0,2);
			pwm_setvalue(v,3);

			break;
	  case 0b010:
			pwm_setvalue(v,1);
			pwm_setvalue(0,2);
			pwm_setvalue(0,3);
			break;
	  case 0b011:
			pwm_setvalue(v,1);
			pwm_setvalue(0,2);
			pwm_setvalue(0,3);

			break;
	  case 0b001:
			pwm_setvalue(0,1);
			pwm_setvalue(v,2);
			pwm_setvalue(0,3);
			break;

	  default:
			pwm_setvalue(0,1);
			pwm_setvalue(0,2);
			pwm_setvalue(0,3);
			break;
	  }
  }else{
	  switch(hall_state){
	  case 0b101:
			pwm_setvalue(-v,1);
			pwm_setvalue(0,2);
			pwm_setvalue(-v,3);
			break;
	  case 0b100:
			pwm_setvalue(-v,1);
			pwm_setvalue(-v,2);
			pwm_setvalue(0,3);
			break;
	  case 0b110:
			pwm_setvalue(-v,1);
			pwm_setvalue(-v,2);
			pwm_setvalue(0,3);
			break;
	  case 0b010:
			pwm_setvalue(0,1);
			pwm_setvalue(-v,2);
			pwm_setvalue(-v,3);
			break;
	  case 0b011:
			pwm_setvalue(0,1);
			pwm_setvalue(-v,2);
			pwm_setvalue(-v,3);
			break;
	  case 0b001:
			pwm_setvalue(-v,1);
			pwm_setvalue(0,2);
			pwm_setvalue(-v,3);
			break;

	  default:
			pwm_setvalue(0,1);
			pwm_setvalue(0,2);
			pwm_setvalue(0,3);
			break;

	  }

  }
//*/

  int n=sprintf(debug_data,"%d,%d,%d,%d,%d\r\n",(int)pulse,hall_state,aADCxConvertedData[0],aADCxConvertedData[1],aADCxConvertedData[2]);
  //int n=sprintf(debug_data,"%d\r\n",(int)pulse);
  Debug(debug_data, n);


  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
