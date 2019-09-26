/**
  ******************************************************************************
  * @file    ts7846.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file provides a set of functions needed to manage the ts7846
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
#include "ts7846.h"
#include "io_revision.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */ 
  
/** @defgroup ts7846
  * @{
  */   
  
/* Private typedef -----------------------------------------------------------*/

/** @defgroup ts7846_Private_Types_Definitions
  * @{
  */ 
 
/* Private define ------------------------------------------------------------*/

/** @defgroup ts7846_Private_Defines
  * @{
  */ 
  
/* Private macro -------------------------------------------------------------*/

/** @defgroup ts7846_Private_Macros
  * @{
  */ 
  
/* Private variables ---------------------------------------------------------*/

/** @defgroup ts7846_Private_Variables
  * @{
  */ 
  
/* Touch screen driver structure initialization */  
TS7846_DrvTypeDef ts7846_ts_drv = 
{
  ts7846_Init,
  ts7846_ReadID,
  ts7846_Reset,
  
  ts7846_TS_Start,
  ts7846_TS_DetectTouch,
  ts7846_TS_GetXY,
  
  ts7846_TS_EnableIT,
  ts7846_TS_ClearIT,
  ts7846_TS_ITStatus,
  ts7846_TS_DisableIT,
};
/* IO driver structure initialization */ 
IO_Revision_DrvTypeDef stmpe811_io_drv = 
{
  stmpe811_IO_Init,
  stmpe811_IO_ReadID,
  stmpe811_IO_Reset,
  
  stmpe811_IO_Start,
  stmpe811_IO_Config,
  stmpe811_IO_WritePin,
  stmpe811_IO_ReadPin,
  
  stmpe811_IO_EnableIT,
  stmpe811_IO_DisableIT,
  stmpe811_IO_ITStatus,
  stmpe811_IO_ClearIT,
};

/**
  * @}
  */ 
    
/* Private function prototypes -----------------------------------------------*/

/** @defgroup ts7846_Private_Function_Prototypes
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup ts7846_Private_Functions
  * @{
  */
/**
  * @brief  Initialize the ts7846 and configure the needed hardware resources
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void ts7846_Init(void)
{
  /* Initialize IO BUS layer */
  //IOE_Init(); 
	TS_Init();

  
}
 
/**
  * @brief  Reset the ts7846 by Software.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void ts7846_Reset(void)
{

}

/**
  * @brief  Read the ts7846 IO Expander device ID.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval The Device ID (two bytes).
  */
uint16_t ts7846_ReadID(void)
{
  return 0;
}

/**
  * @brief  Configures the touch Screen Controller (Single point detection)
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None.
  */
void ts7846_TS_Start(void)
{
}

/**
  * @brief  Initialize the ts7846 and configure the needed hardware resources
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void stmpe811_IO_Init(void)
{
  /* Initialize IO BUS layer */
  //IOE_Init();
//	TS_Init();


}

/**
  * @brief  Read the ts7846 IO Expander device ID.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval The Device ID (two bytes).
  */
uint16_t stmpe811_IO_ReadID(void)
{
  return 0;
}

/**
  * @brief  Reset the ts7846 by Software.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval None
  */
void stmpe811_IO_Reset(void)
{

}
/**
  * @brief  Start the IO functionality use and enable the AF for selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The IO pin(s) to put in AF. This parameter can be one 
  *         of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @retval None
  */
void stmpe811_IO_Start(uint16_t DeviceAddr, uint16_t IO_Pin)
{
 
}

/**
  * @brief  Configures the IO pin(s) according to IO mode structure value.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The output pin to be set or reset. This parameter can be one 
  *         of the following values:   
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @param  IO_Mode: The IO pin mode to configure, could be one of the following values:
  *   @arg  IO_MODE_INPUT
  *   @arg  IO_MODE_OUTPUT
  *   @arg  IO_MODE_IT_RISING_EDGE
  *   @arg  IO_MODE_IT_FALLING_EDGE
  *   @arg  IO_MODE_IT_LOW_LEVEL
  *   @arg  IO_MODE_IT_HIGH_LEVEL            
  * @retval None
  */
void stmpe811_IO_Config(uint16_t DeviceAddr, uint16_t IO_Pin, IO_ModeTypedef IO_Mode)
{
 
  
  
}

/**
  * @brief  Initialize the selected IO pin direction.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.   
  * @param  Direction: could be STMPE811_DIRECTION_IN or STMPE811_DIRECTION_OUT.      
  * @retval None
  */
void stmpe811_IO_InitPin(uint16_t DeviceAddr, uint16_t IO_Pin, uint8_t Direction)
{

}

/**
  * @brief  Enable the AF for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.        
  * @retval None
  */
void stmpe811_IO_EnableAF(uint16_t DeviceAddr, uint16_t IO_Pin)
{
  
}

/**
  * @brief  Disable the AF for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.       
  * @retval None
  */
void stmpe811_IO_DisableAF(uint16_t DeviceAddr, uint16_t IO_Pin)
{
  
  
}

/**
  * @brief  Configure the Edge for which a transition is detectable for the
  *         selected pin.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO pin to be configured. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.  
  * @param  Edge: The edge which will be detected. This parameter can be one or
  *         a combination of following values: STMPE811_EDGE_FALLING and STMPE811_EDGE_RISING .
  * @retval None
  */
void stmpe811_IO_SetEdgeMode(uint16_t DeviceAddr, uint16_t IO_Pin, uint8_t Edge)
{

}

  
 

/**
  * @brief  Write a new IO pin state.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param IO_Pin: The output pin to be set or reset. This parameter can be one 
  *        of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7. 
  * @param PinState: The new IO pin state.
  * @retval None
  */
void stmpe811_IO_WritePin(uint16_t DeviceAddr, uint16_t IO_Pin, uint8_t PinState)
{
 
}

/**
  * @brief  Return the state of the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.  
  * @param IO_Pin: The output pin to be set or reset. This parameter can be one 
  *        of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7. 
  * @retval IO pin(s) state.
  */
uint16_t stmpe811_IO_ReadPin(uint16_t DeviceAddr, uint16_t IO_Pin)
{
  
}

/**
  * @brief  Enable the global IO interrupt source.
  * @param  DeviceAddr: Device address on communication Bus.  
  * @retval None
  */
void stmpe811_IO_EnableIT(uint16_t DeviceAddr)
{ 
  IOE_ITConfig();
}

/**
  * @brief  Disable the global IO interrupt source.
  * @param  DeviceAddr: Device address on communication Bus.   
  * @retval None
  */
void stmpe811_IO_DisableIT(uint16_t DeviceAddr)
{
 
}

/**
  * @brief  Enable interrupt mode for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be enabled. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @retval None
  */
void stmpe811_IO_EnablePinIT(uint16_t DeviceAddr, uint16_t IO_Pin)
{
 
}

/**
  * @brief  Disable interrupt mode for the selected IO pin(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be disabled. This parameter could be any 
  *         combination of the following values:
  *   @arg  STMPE811_PIN_x: where x can be from 0 to 7.
  * @retval None
  */
void stmpe811_IO_DisablePinIT(uint16_t DeviceAddr, uint16_t IO_Pin)
{
 
}

/**
  * @brief  Check the status of the selected IO interrupt pending bit
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: The IO interrupt to be checked could be:
  *   @arg  STMPE811_PIN_x Where x can be from 0 to 7.             
  * @retval Status of the checked IO pin(s).
  */
uint8_t stmpe811_IO_ITStatus(uint16_t DeviceAddr, uint16_t IO_Pin)
{
 
}

/**
  * @brief  Clear the selected IO interrupt pending bit(s).
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  IO_Pin: the IO interrupt to be cleared, could be:
  *   @arg  STMPE811_PIN_x: Where x can be from 0 to 7.            
  * @retval None
  */
void stmpe811_IO_ClearIT(uint16_t DeviceAddr, uint16_t IO_Pin)
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
