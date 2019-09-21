/**
  ******************************************************************************
  * @file    stm324x9i_eval.h
  * @author  MCD Application Team
  * @version V2.0.2
  * @date    19-June-2014
  * @brief   This file contains definitions for STM324x9I_EVAL's LEDs, 
  *          push-buttons and COM ports hardware resources.
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
#ifndef __STM324X9I_EVAL_H
#define __STM324X9I_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM324x9I_EVAL
  * @{
  */
      
/** @addtogroup STM324x9I_EVAL_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3
}Led_TypeDef;

typedef enum 
{  
  BUTTON_WAKEUP = 0,
  BUTTON_TAMPER = 1,
  BUTTON_KEY = 2
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

typedef enum 
{  
  JOY_MODE_GPIO = 0,
  JOY_MODE_EXTI = 1
}JOYMode_TypeDef;

typedef enum 
{ 
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
}JOYState_TypeDef;

typedef enum 
{
  COM1 = 0,
  COM2 = 1
}COM_TypeDef;
/**
  * @}
  */ 

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** 
  * @brief  Define for STM324x9I_EVAL board  
  */ 
#if !defined (USE_STM324x9I_EVAL)
 #define USE_STM324x9I_EVAL
#endif

/** @addtogroup STM324x9I_EVAL_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             4

#define LED1_PIN                         GPIO_PIN_13
#define LED1_GPIO_PORT                   GPIOG
#define LED1_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()  
#define LED1_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()  
    
    
#define LED2_PIN                         GPIO_PIN_14
#define LED2_GPIO_PORT                   GPIOG
#define LED2_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()   
#define LED2_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()  
  
#define LED3_PIN                         GPIO_PIN_10
#define LED3_GPIO_PORT                   GPIOG
#define LED3_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()   
#define LED3_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()  
  
#define LED4_PIN                         GPIO_PIN_12
#define LED4_GPIO_PORT                   GPIOG
#define LED4_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()   
#define LED4_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()  
    
#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? LED1_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_ENABLE() : LED4_GPIO_CLK_ENABLE())

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED1_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_DISABLE() : LED4_GPIO_CLK_DISABLE())

/**
  * @}
  */ 
  
/** @addtogroup STM324x9I_EVAL_LOW_LEVEL_BUTTON
  * @{
  */ 
/* Joystick pins are connected to IO Expander (accessible through I2C1 interface) */ 
#define BUTTONn                             3 

/**
  * @brief Wakeup push-button
  */
#define WAKEUP_BUTTON_PIN                   GPIO_PIN_0
#define WAKEUP_BUTTON_GPIO_PORT             GPIOA
#define WAKEUP_BUTTON_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()  
#define WAKEUP_BUTTON_GPIO_CLK_DISABLE()    __GPIOA_CLK_DISABLE()
#define WAKEUP_BUTTON_EXTI_IRQn             EXTI0_IRQn 

/**
  * @brief Tamper push-button
  */
#define TAMPER_BUTTON_PIN                    GPIO_PIN_13
#define TAMPER_BUTTON_GPIO_PORT              GPIOC
#define TAMPER_BUTTON_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()  
#define TAMPER_BUTTON_GPIO_CLK_DISABLE()     __GPIOC_CLK_DISABLE()
#define TAMPER_BUTTON_EXTI_IRQn              EXTI15_10_IRQn

/**
  * @brief Key push-button
  */
#define KEY_BUTTON_PIN                       GPIO_PIN_13
#define KEY_BUTTON_GPIO_PORT                 GPIOC
#define KEY_BUTTON_GPIO_CLK_ENABLE()         __GPIOC_CLK_ENABLE()  
#define KEY_BUTTON_GPIO_CLK_DISABLE()        __GPIOC_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn                 EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? WAKEUP_BUTTON_GPIO_CLK_ENABLE() :\
                                               ((__INDEX__) == 1) ? TAMPER_BUTTON_GPIO_CLK_ENABLE() : KEY_BUTTON_GPIO_CLK_ENABLE())

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    (((__INDEX__) == 0) ? WAKEUP_BUTTON_GPIO_CLK_DISABLE() :\
                                                ((__INDEX__) == 1) ? TAMPER_BUTTON_GPIO_CLK_DISABLE() : KEY_BUTTON_GPIO_CLK_DISABLE())
/**
  * @}
  */ 

/** @addtogroup STM324x9I_EVAL_LOW_LEVEL_COM
  * @{
  */
#define COMn                             1

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define EVAL_COM1                          USART1
#define EVAL_COM1_CLK_ENABLE()             __USART1_CLK_ENABLE()   
#define EVAL_COM1_CLK_DISABLE()            __USART1_CLK_DISABLE()

#define EVAL_COM1_TX_PIN                   GPIO_PIN_9
#define EVAL_COM1_TX_GPIO_PORT             GPIOA
#define EVAL_COM1_TX_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()   
#define EVAL_COM1_TX_GPIO_CLK_DISABLE()    __GPIOA_CLK_DISABLE()  
#define EVAL_COM1_TX_AF                    GPIO_AF7_USART1

#define EVAL_COM1_RX_PIN                   GPIO_PIN_10
#define EVAL_COM1_RX_GPIO_PORT             GPIOA
#define EVAL_COM1_RX_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()   
#define EVAL_COM1_RX_GPIO_CLK_DISABLE()    __GPIOA_CLK_DISABLE()  
#define EVAL_COM1_RX_AF                    GPIO_AF7_USART1

#define EVAL_COM1_IRQn                     USART1_IRQn

#define EVAL_COMx_CLK_ENABLE(__INDEX__)            (((__INDEX__) == 0) ? EVAL_COM1_CLK_ENABLE() : 0)
#define EVAL_COMx_CLK_DISABLE(__INDEX__)           (((__INDEX__) == 0) ? EVAL_COM1_CLK_DISABLE() : 0)

#define EVAL_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? EVAL_COM1_TX_GPIO_CLK_ENABLE() : 0)
#define EVAL_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_TX_GPIO_CLK_DISABLE() : 0)

#define EVAL_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? EVAL_COM1_RX_GPIO_CLK_ENABLE() : 0)
#define EVAL_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_RX_GPIO_CLK_DISABLE() : 0)

/**
  * @brief Joystick Pins definition 
  */ 
#define JOY_SEL_PIN                    IO_PIN_14
#define JOY_DOWN_PIN                   IO_PIN_13
#define JOY_LEFT_PIN                   IO_PIN_12
#define JOY_RIGHT_PIN                  IO_PIN_11
#define JOY_UP_PIN                     IO_PIN_10
#define JOY_NONE_PIN                   JOY_ALL_PINS
#define JOY_ALL_PINS                   (IO_PIN_10 | IO_PIN_11 | IO_PIN_12 | IO_PIN_13 | IO_PIN_14)

/**
  * @brief Eval Pins definition 
  */
#define XSDN_PIN                       IO_PIN_0
#define MII_INT_PIN                    IO_PIN_1
#define RSTI_PIN                       IO_PIN_2
#define CAM_PLUG_PIN                   IO_PIN_3
#define LCD_INT_PIN                    IO_PIN_4
#define AUDIO_INT_PIN                  IO_PIN_5
#define OTG_FS1_OVER_CURRENT_PIN       IO_PIN_6
#define OTG_FS1_POWER_SWITCH_PIN       IO_PIN_7
#define OTG_FS2_OVER_CURRENT_PIN       IO_PIN_8
#define OTG_FS2_POWER_SWITCH_PIN       IO_PIN_9 
//#define SD_DETECT_PIN                  IO_PIN_15

/* Exported constant IO ------------------------------------------------------*/
#define IO_I2C_ADDRESS                   0x84 
#define TS_I2C_ADDRESS                   0x82
#define TS3510_I2C_ADDRESS               0x80
#define CAMERA_I2C_ADDRESS               0x60
#define AUDIO_I2C_ADDRESS                0x34
#define EEPROM_I2C_ADDRESS_A01           0xA0
#define EEPROM_I2C_ADDRESS_A02           0xA6  
/* I2C clock speed configuration (in Hz) 
   WARNING: 
   Make sure that this define is not already declared in other files (ie. 
   stm324x9I_eval.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED                        100000
#endif /* I2C_SPEED */

/* User can use this section to tailor I2Cx/I2Cx instance used and associated 
   resources */
/* Definition for I2Cx clock resources */
#define EVAL_I2Cx                             I2C1
#define EVAL_I2Cx_CLK_ENABLE()                __I2C1_CLK_ENABLE()
#define EVAL_DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
#define EVAL_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()

#define EVAL_I2Cx_FORCE_RESET()               __I2C1_FORCE_RESET()
#define EVAL_I2Cx_RELEASE_RESET()             __I2C1_RELEASE_RESET()
   
/* Definition for I2Cx Pins */
#define EVAL_I2Cx_SCL_PIN                     GPIO_PIN_6
#define EVAL_I2Cx_SCL_SDA_GPIO_PORT           GPIOB
#define EVAL_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C1
#define EVAL_I2Cx_SDA_PIN                     GPIO_PIN_9

/* I2C interrupt requests */
#define EVAL_I2Cx_EV_IRQn                     I2C1_EV_IRQn
#define EVAL_I2Cx_ER_IRQn                     I2C1_ER_IRQn

//Touch Screen PIN---------------------------------------
//////////////////////////////////TOUCH PANEL PIN////////////////////////////////////////////
//硬件相关的子函数

#define	CHX 	0x90 	//通道Y+的选择控制字	
#define	CHY 	0xD0	//通道X+的选择控制字 

#define PIN_TP_DCLK 	13		//SCK	O
#define PORT_TP_DCLK 	GPIOB
#define PIN_TP_CS 		5		//NSS	O
#define PORT_TP_CS 		GPIOB
#define PIN_TP_DIN 		15		//MOSI	O
#define PORT_TP_DIN 	GPIOB
#define PIN_TP_DOUT 	2		//MISO	I
#define PORT_TP_DOUT 	GPIOC
#define PIN_TP_INT		14		//IRQ	I
#define PORT_TP_INT		GPIOB
#define PIN_TP_BUSY		12		//BUSY	I
#define PORT_TP_BUSY	GPIOB

#define TP_DCLK(a)	\
						if (!a)	\
						PORT_TP_DCLK->BSRRH |= 1UL << PIN_TP_DCLK;	\
						else		\
						PORT_TP_DCLK->BSRRL |= 1UL << PIN_TP_DCLK
#define TP_CS(a)	\
						if (!a)	\
						PORT_TP_CS->BSRRH |= 1UL << PIN_TP_CS;	\
						else		\
						PORT_TP_CS->BSRRL |= 1UL << PIN_TP_CS
#define TP_DIN(a)	\
						if (!a)	\
						PORT_TP_DIN->BSRRH |= 1UL << PIN_TP_DIN;	\
						else		\
						PORT_TP_DIN->BSRRL |= 1UL << PIN_TP_DIN
													
#define TP_DOUT		(PORT_TP_DOUT->IDR & (1UL << PIN_TP_DOUT)) >> PIN_TP_DOUT
#define TP_BUSY		(PORT_TP_BUSY->IDR & (1UL << PIN_TP_BUSY)) >> PIN_TP_BUSY
#define TP_INT_IN   (PORT_TP_INT->IDR & (1UL << PIN_TP_INT)) >> PIN_TP_INT
						
						
/*****************************ADS7846 SPI2********************************************/
/* Definition for SPIx clock resources */	
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __SPI2_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE() 

#define SPIx_FORCE_RESET()               __SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __SPI2_RELEASE_RESET()			

/* Definition for SPIx Pins */
#define TP_SPIx_SCK_PIN                  GPIO_PIN_13
#define TP_SPIx_SCK_GPIO_PORT            GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI2
#define TP_SPIx_MISO_PIN                 GPIO_PIN_2
#define TP_SPIx_MISO_GPIO_PORT           GPIOC
#define SPIx_MISO_AF                     GPIO_AF5_SPI2
#define TP_SPIx_MOSI_PIN                 GPIO_PIN_15
#define TP_SPIx_MOSI_GPIO_PORT           GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI2

/*Definition GPIO */
#define TP_CS_PIN												 GPIO_PIN_5
#define TP_CS_CLK_ENABLE()							 __GPIOB_CLK_ENABLE()	
#define TP_CS_PORT											 GPIOB		

#define TP_BUSY_PIN											 GPIO_PIN_12	
#define TP_BUSY_CLK_ENABLE()						 __GPIOB_CLK_ENABLE()							
#define TP_BUSY_PORT										 GPIOB

#define TP_IRQ_PIN											GPIO_PIN_14
#define TP_IRQ_CLK_ENABLE()							__GPIOB_CLK_ENABLE()
#define TP_IRQ_PORT											GPIOB
#define TP_IRQn													EXTI15_10_IRQn

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI2_IRQn
#define SPIx_IRQHandler                  SPI2_IRQHandler						
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);
void             BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *husart);
uint8_t          BSP_JOY_Init(JOYMode_TypeDef Joy_Mode);
JOYState_TypeDef BSP_JOY_GetState(void);

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

#ifdef __cplusplus
}
#endif

#endif /* __STM324X9I_EVAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
