/**
  ******************************************************************************
  * @file    TIM/TIM_7PWMOutput/Src/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   This sample code shows how to use STM32F4xx TIM HAL API to generate
  *          7 signals in PWM.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "k_lcd_brightness.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_7PWMOutput
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Period value */
extern uint32_t uwPeriod;

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef TimOcHandle;

/* Private function prototypes -----------------------------------------------*/
static void TIM1_Config(void);
static void PWM_Channel_Config(void);
static void TIM1_MspInit(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void LCD_Brightness(uint8_t Duty){
	uint32_t PulseWidth;
	  /* Compute Pulse1 value to generate a duty cycle at Duty for channel 4 */
  PulseWidth = (Duty * (uwPeriod - 1)) / 100;
	TimOcHandle.Pulse = PulseWidth;
	HAL_TIM_PWM_ConfigChannel(&TimHandle,&TimOcHandle,TIM_CHANNEL_4);
	  /* Start channel 4 */
  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4);
}
void PWM_Init(void){
	TIM1_Config();
	PWM_Channel_Config();
}
static void TIM1_Config(void){
	  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* Initialize TIMx peripheral as follow:
       + Prescaler = 0
       + Period = uwPeriod  (to have an output frequency equal to 17.57 KHz)
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Instance = TIM1;
  
  TimHandle.Init.Period        = uwPeriod;
  TimHandle.Init.Prescaler     = 0;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
	
	TIM1_MspInit();
	HAL_TIM_PWM_Init(&TimHandle);
}
static void PWM_Channel_Config(void){
	  /*##-2- Configure the PWM channels #########################################*/ 
  /* Common configuration for all channels */
  TimOcHandle.OCMode      = TIM_OCMODE_PWM2;
  TimOcHandle.OCFastMode  = TIM_OCFAST_DISABLE;
  TimOcHandle.OCPolarity  = TIM_OCPOLARITY_LOW;
  TimOcHandle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  TimOcHandle.OCIdleState = TIM_OCIDLESTATE_SET;
  TimOcHandle.OCNIdleState= TIM_OCNIDLESTATE_RESET;	
}
static void TIM1_MspInit(void){
  GPIO_InitTypeDef   GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIM1 Peripheral clock enable */
  __TIM1_CLK_ENABLE();
    
  /* Enable GPIO Port Clocks */
  __GPIOA_CLK_ENABLE();
  
  /* Common configuration for all channels */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  
  /* Channel 4 output */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
}