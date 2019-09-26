/**
  ******************************************************************************
  * @file    ad7846.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file contains all the functions prototypes for the
  *          ad7846.c IO expander driver.
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
#ifndef __ad7846_H
#define __ad7846_H

#ifdef __cplusplus
 extern "C" {
#endif   
   
/* Includes ------------------------------------------------------------------*/
#include "..\Common\ts_7846.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Component
  * @{
  */
    
/** @defgroup ad7846
  * @{
  */    

/* Exported types ------------------------------------------------------------*/

/** @defgroup ad7846_Exported_Types
  * @{
  */ 

/* Exported constants --------------------------------------------------------*/
  
/** @defgroup ad7846_Exported_Constants
  * @{
  */ 

/*  */   
#define ad7846_READ_BLOCK_REG                     0x8A
#define ad7846_SEND_CMD_REG                       0x00
#define ad7846_READ_CMD                           0x81  
#define ad7846_WRITE_CMD                          0x08  



///////////////////////////////////////
//=========================================================================================================================================
//#define THRESHOLD 2
//兼容ADS7843 XPT2046

// A/D 通道选择命令字和工作寄存器
//#define	CHX 	0x90 	//通道Y+的选择控制字	
//#define	CHY 	0xd0	//通道X+的选择控制字 


/**
  * @}
  */ 
  
/* Exported macro ------------------------------------------------------------*/
   
/** @defgroup ad7846_Exported_Macros
  * @{
  */ 

/* Exported functions --------------------------------------------------------*/
  
/** @defgroup ad7846_Exported_Functions
  * @{
  */

/** 
  * @brief ad7846 Control functions
  */
void     ad7846_Init(void);
void     ad7846_Reset(void);
uint16_t ad7846_ReadID(void);
void     ad7846_TS_Start(void);
uint8_t  ad7846_TS_DetectTouch(void);
void     ad7846_TS_GetXY(uint16_t *X, uint16_t *Y);
void     ad7846_TS_EnableIT(void);
void     ad7846_TS_DisableIT(void);
uint8_t  ad7846_TS_ITStatus (void);
void     ad7846_TS_ClearIT (void);

void 		 TS2_Init(void);
void		 TS2_EnableIT(void);
void		 TS2_ClearIT(void);
uint32_t TS2_DetectTouch(void);
uint16_t TS2_Read_X(void);
uint16_t TS2_Read_Y(void);
void		 TS2_Read_ADX(void);
void		 TS2_Read_ADY(void);
void     TS2_Delayus(uint32_t delay);
uint8_t  IOE_Read(uint8_t addr, uint8_t reg);
uint16_t IOE_ReadMultiple(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);
void     IOE_WriteMultiple(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);

/* Touch screen driver structure */
extern TS7846_DrvTypeDef ad7846_ts_drv;

#ifdef __cplusplus
}
#endif
#endif /* __ad7846_H */


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
