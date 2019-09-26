/**
  ******************************************************************************
  * @file    io.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    21-March-2014
  * @brief   This file contains all the functions prototypes for the IO driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IO_H
#define __IO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */
    
/** @addtogroup IO
  * @{
  */

/** @defgroup IO_Exported_Types
  * @{
  */
typedef enum
{
   IO_MODE_INPUT = 0,
   IO_MODE_OUTPUT,
   IO_MODE_IT_RISING_EDGE,
   IO_MODE_IT_FALLING_EDGE,
   IO_MODE_IT_LOW_LEVEL,
   IO_MODE_IT_HIGH_LEVEL
   
}IO_ModeTypedef;

typedef struct
{  
  void       (*Init)(void);
  uint16_t   (*ReadID)(void);
  void       (*Reset)(void);
  
  void       (*Start)(uint16_t, uint16_t);
  void       (*Config)(uint16_t, uint16_t, IO_ModeTypedef);
  void       (*WritePin)(uint16_t, uint16_t, uint8_t);
  uint16_t   (*ReadPin)(uint16_t, uint16_t);
  
  void       (*EnableIT)(uint16_t);
  void       (*DisableIT)(uint16_t);
  uint8_t    (*ITStatus)(uint16_t, uint16_t);
  void       (*ClearIT)(uint16_t, uint16_t);
    
}IO_Revision_DrvTypeDef;
/**
  * @}
  */
/** 
  * @brief STMPE811 IO functionalities functions
  */
void     stmpe811_IO_Init(void);
void     stmpe811_IO_Reset(void);
uint16_t stmpe811_IO_ReadID(void);
void     stmpe811_IO_Start(uint16_t DeviceAddr, uint16_t IO_Pin);
void     stmpe811_IO_Config(uint16_t DeviceAddr, uint16_t IO_Pin, IO_ModeTypedef IO_Mode);
void     stmpe811_IO_InitPin(uint16_t DeviceAddr, uint16_t IO_Pin, uint8_t Direction);
void     stmpe811_IO_EnableAF(uint16_t DeviceAddr, uint16_t IO_Pin);
void     stmpe811_IO_DisableAF(uint16_t DeviceAddr, uint16_t IO_Pin);
void     stmpe811_IO_SetEdgeMode(uint16_t DeviceAddr, uint16_t IO_Pin, uint8_t Edge);
void     stmpe811_IO_WritePin(uint16_t DeviceAddr, uint16_t IO_Pin, uint8_t PinState);
uint16_t stmpe811_IO_ReadPin(uint16_t DeviceAddr, uint16_t IO_Pin);
void     stmpe811_IO_EnableIT(uint16_t DeviceAddr);
void     stmpe811_IO_DisableIT(uint16_t DeviceAddr);
void     stmpe811_IO_EnablePinIT(uint16_t DeviceAddr, uint16_t IO_Pin);
void     stmpe811_IO_DisablePinIT(uint16_t DeviceAddr, uint16_t IO_Pin);
uint8_t  stmpe811_IO_ITStatus(uint16_t DeviceAddr, uint16_t IO_Pin);
void     stmpe811_IO_ClearIT(uint16_t DeviceAddr, uint16_t IO_Pin);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __IO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
