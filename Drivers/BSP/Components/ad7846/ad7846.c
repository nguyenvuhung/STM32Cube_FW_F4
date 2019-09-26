/**
  ******************************************************************************
  * @file    ad7846.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file provides a set of functions needed to manage the ad7846
  *          IO Expander devices.
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
#include "ad7846.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */ 
  
/** @defgroup ad7846
  * @{
  */   
  
/* Private typedef -----------------------------------------------------------*/

/** @defgroup ad7846_Private_Types_Definitions
  * @{
  */ 
 
/* Private define ------------------------------------------------------------*/

/** @defgroup ad7846_Private_Defines
  * @{
  */ 
  
/* Private macro -------------------------------------------------------------*/

/** @defgroup ad7846_Private_Macros
  * @{
  */ 
  
/* Private variables ---------------------------------------------------------*/

/** @defgroup ad7846_Private_Variables
  * @{
  */ 
/* Touch screen driver structure initialization */  
TS7846_DrvTypeDef ad7846_ts_drv = 
{
  ad7846_Init,
  ad7846_ReadID,
  ad7846_Reset,
  
  ad7846_TS_Start,
  ad7846_TS_DetectTouch,
  ad7846_TS_GetXY,
  
  ad7846_TS_EnableIT,
  ad7846_TS_ClearIT,
  ad7846_TS_ITStatus,
  ad7846_TS_DisableIT,
};

/**
  * @}
  */ 
    
/* Private function prototypes -----------------------------------------------*/

/** @defgroup ad7846_Private_Function_Prototypes
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup ad7846_Private_Functions
  * @{
  */
/**
  * @brief  Initialize the ad7846 and configure the needed hardware resources
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void ad7846_Init(void)
{
  /* Initialize IO BUS layer */
	TS2_Init();

}
 
/**
  * @brief  Reset the ad7846 by Software.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void ad7846_Reset(void)
{

}

/**
  * @brief  Read the ad7846 IO Expander device ID.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval The Device ID (two bytes).
  */
uint16_t ad7846_ReadID(void)
{
  return 0;
}

/**
  * @brief  Configures the touch Screen Controller (Single point detection)
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None.
  */
void ad7846_TS_Start(void)
{
}

/**
  * @brief  Return if there is touch detected or not.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Touch detected state.
  */
uint8_t ad7846_TS_DetectTouch(void)
{
  return (uint8_t)TS2_DetectTouch();
}

/**
  * @brief  Get the touch screen X and Y positions values
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value   
  * @retval None.
  */
void ad7846_TS_GetXY(uint16_t *X, uint16_t *Y)
{
    uint16_t adx,ady;
    adx=TS2_Read_X();
    ady=TS2_Read_Y();
    *X=adx;
    *Y=ady;
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void ad7846_TS_EnableIT(void)
{  
	TS2_EnableIT();
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.    
  * @retval None
  */
void ad7846_TS_DisableIT(void)
{
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.    
  * @retval TS interrupts status
  */
uint8_t ad7846_TS_ITStatus(void)
{
  return 0;
}

/**
  * @brief  Configure the selected source to generate a global interrupt or not
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void ad7846_TS_ClearIT(void)
{
}
/**
  * @}
  */ 
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */      
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
