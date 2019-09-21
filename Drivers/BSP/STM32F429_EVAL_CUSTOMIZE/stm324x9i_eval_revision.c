/**
  ******************************************************************************
  * @file    stm324x9i_eval.c
  * @author  MCD Application Team
  * @version V2.0.2
  * @date    19-June-2014
  * @brief   This file provides a set of firmware functions to manage LEDs, 
  *          push-buttons and COM ports available on STM324x9I-EVAL evaluation 
  *          board(MB1045) RevB from STMicroelectronics.
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
  
/* File Info: ------------------------------------------------------------------
                                   User NOTE

   This driver requires the stm324x9i_eval_io to manage the joystick

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm324x9i_eval_revision.h"
#include "stm324x9i_eval_io.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM324x9I_EVAL
  * @{
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL 
  * @{
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Defines
  * @{
  */
	/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/**
 * @brief STM324x9I EVAL BSP Driver version number V2.0.2
   */
#define __STM324x9I_EVAL_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM324x9I_EVAL_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM324x9I_EVAL_BSP_VERSION_SUB2   (0x02) /*!< [15:8]  sub2 version */
#define __STM324x9I_EVAL_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __STM324x9I_EVAL_BSP_VERSION         ((__STM324x9I_EVAL_BSP_VERSION_MAIN << 24)\
                                             |(__STM324x9I_EVAL_BSP_VERSION_SUB1 << 16)\
                                             |(__STM324x9I_EVAL_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM324x9I_EVAL_BSP_VERSION_RC))
																						 
																						 
#define LCD_BASE   (0x60000000UL)
#define LCD_CMD  (*((volatile unsigned short *)(LCD_BASE)))
#define LCD_DATA (*((volatile unsigned short *)(LCD_BASE+0x00100000UL)))																					
/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Variables
  * @{
  */
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT,
                                 LED2_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 LED2_PIN,
                                 LED3_PIN,
                                 LED4_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {WAKEUP_BUTTON_GPIO_PORT,
                                      TAMPER_BUTTON_GPIO_PORT,
                                      KEY_BUTTON_GPIO_PORT};

const uint16_t BUTTON_PIN[BUTTONn] = {WAKEUP_BUTTON_PIN,
                                      TAMPER_BUTTON_PIN,
                                      KEY_BUTTON_PIN};

const uint16_t BUTTON_IRQn[BUTTONn] = {WAKEUP_BUTTON_EXTI_IRQn,
                                       TAMPER_BUTTON_EXTI_IRQn,
                                       KEY_BUTTON_EXTI_IRQn};

USART_TypeDef* COM_USART[COMn] = {EVAL_COM1};

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN};

const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF};

const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF};

static I2C_HandleTypeDef heval_I2c;
static uint8_t Is_LCD_IO_Initialized = 0;

SPI_HandleTypeDef SpiHandle;

/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */
static void     I2Cx_MspInit(void);
static void     I2Cx_Init(void);
static void     I2Cx_ITConfig(void);
static void     I2Cx_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t  I2Cx_Read(uint8_t Addr, uint8_t Reg);
static HAL_StatusTypeDef I2Cx_ReadMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_WriteMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
static void     I2Cx_Error(uint8_t Addr);

/* IOExpander IO functions */
void            IOE_Init(void);
void            IOE_ITConfig(void);
void            IOE_Delay(uint32_t Delay);
void            IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t        IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void            IOE_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

/* AUDIO IO functions */
void            AUDIO_IO_Init(void);
void            AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
uint16_t        AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
void            AUDIO_IO_Delay(uint32_t Delay);

/* CAMERA IO functions */
void            CAMERA_IO_Init(void);
void            CAMERA_Delay(uint32_t Delay);
void            CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         CAMERA_IO_Read(uint8_t Addr, uint8_t Reg);

/* I2C EEPROM IO function */
void                EEPROM_IO_Init(void);
HAL_StatusTypeDef   EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef   EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef   EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);

//-------------Touch Screen--------------------------------------------------------------------------------
void TS_Init(void);
uint32_t TS_DetectTouch (void) ;
uint16_t TS_Read_X(void) ;
uint16_t TS_Read_Y(void) ;
void TS_Read_ADX(void);
void TS_Read_ADY(void);
static uint16_t TS_ReadAD(void) ;
static void TS_WriteCMD(unsigned char cmd) ;
static void TS_Delayus( int k);
/**********************ADS7843 TOUCH PANEL********************************************************/
void TS2_Init(void);
uint32_t TS2_DetectTouch (void) ;
void TS2_EnableIT(void);
uint16_t TS2_Read_X(void) ;
uint16_t TS2_Read_Y(void) ;
static void SPI_MspInit(void);
static void SPI_MspDeInit(void);
static void TP2_CS(uint8_t State);
static void TP2_CLK(uint8_t State);
static void TP2_DIN(uint8_t State);
static void TP2_DOUT(uint8_t State);
static uint8_t SPI_ShiftByte(uint8_t TxData);
static void SpiDelay(unsigned int DelayCnt);

void 				TS3_Init(void);
uint16_t	  TS3_Read_X(void);
uint16_t	  TS3_Read_Y(void);
uint32_t TS3_DetectTouch(void);
static void TP3_CS(uint8_t State);
static void TP3_CLK(uint8_t State);
static void TP3_DIN(uint8_t State);
static void TP3_DOUT(uint8_t State);
static void TS3_WriteCmd(uint8_t Cmd);
static uint16_t TS3_Read_AD(uint8_t CMD);

/*FMC IO Function*/
static void     FMC_BANK1_WriteData(uint16_t Data);
static void     FMC_BANK1_WriteReg(uint8_t Reg);
static uint16_t FMC_BANK1_ReadData(void);
static void     FMC_BANK1_Init(void);
static void     FMC_BANK1_MspInit(void);



/* LCD IO functions */
void            LCD_IO_Init(void);
void            LCD_IO_WriteData(uint16_t RegValue);
void            LCD_IO_WriteReg(uint8_t Reg);
uint16_t        LCD_IO_ReadData(void);
/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Functions
  * @{
  */ 

  /**
  * @brief  This method returns the STM324x9I EVAL BSP Driver revision
  * @param  None
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM324x9I_EVAL_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: LED to be configured. 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: LED to be set on 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: LED to be set off
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: LED to be toggled
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief  Configures button GPIO and EXTI Line.
  * @param  Button: Button to be configured
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_WAKEUP: Wakeup Push Button 
  *            @arg  BUTTON_TAMPER: Tamper Push Button  
  * @param  Button_Mode: Button mode
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *            @arg  BUTTON_MODE_EXTI: Button will be connected to EXTI line 
  *                                    with interrupt generation capability  
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the BUTTON clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  if(Button_Mode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
  
  if(Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    
    if(Button != BUTTON_WAKEUP)
    {
      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; 
    }
    else
    {
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    }
    
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  Returns the selected button state.
  * @param  Button: Button to be checked
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_WAKEUP: Wakeup Push Button 
  *            @arg  BUTTON_TAMPER: Tamper Push Button 
  *            @arg  BUTTON_KEY: Key Push Button
  * @retval The Button GPIO pin value
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Configures COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1 
  *            @arg  COM2 
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  * @retval None
  */
void BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIO clock */
  EVAL_COMx_TX_GPIO_CLK_ENABLE(COM);
  EVAL_COMx_RX_GPIO_CLK_ENABLE(COM);

  /* Enable USART clock */
  EVAL_COMx_CLK_ENABLE(COM);

  /* Configure USART Tx as alternate function */
  GPIO_InitStruct.Pin = COM_TX_PIN[COM];
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = COM_TX_AF[COM];
  HAL_GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStruct);

  /* Configure USART Rx as alternate function */
  GPIO_InitStruct.Pin = COM_RX_PIN[COM];
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = COM_RX_AF[COM];
  HAL_GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStruct);

  /* USART configuration */
  huart->Instance = COM_USART[COM];
  HAL_UART_Init(huart);
}

/**
  * @brief  Configures joystick GPIO and EXTI modes.
  * @param  Joy_Mode: Button mode.
  *          This parameter can be one of the following values:
  *            @arg  JOY_MODE_GPIO: Joystick pins will be used as simple IOs
  *            @arg  JOY_MODE_EXTI: Joystick pins will be connected to EXTI line 
  *                                 with interrupt generation capability  
  * @retval IO_OK: if all initializations are OK. Other value if error.
  */
uint8_t BSP_JOY_Init(JOYMode_TypeDef Joy_Mode)
{
  uint8_t ret = 0;
  
  /* Initialize the IO functionalities */
  ret = BSP_IO_Init();
  
  /* Configure joystick pins in IT mode */
  if(Joy_Mode == JOY_MODE_EXTI)
  {
    /* Configure IO interrupt acquisition mode */
    BSP_IO_ConfigPin(JOY_ALL_PINS, IO_MODE_IT_FALLING_EDGE);
  }
  
  return ret; 
}

/**
  * @brief  Returns the current joystick status.
  * @param  None
  * @retval Code of the joystick key pressed
  *          This code can be one of the following values:
  *            @arg  JOY_NONE
  *            @arg  JOY_SEL
  *            @arg  JOY_DOWN
  *            @arg  JOY_LEFT
  *            @arg  JOY_RIGHT
  *            @arg  JOY_UP
  */
JOYState_TypeDef BSP_JOY_GetState(void)
{
  uint16_t tmp = 0;   
  
  /* Read the status joystick pins */
  tmp = BSP_IO_ReadPin(JOY_ALL_PINS);
   
  /* Check the pressed keys */  
  if((tmp & JOY_NONE_PIN) == JOY_NONE)
  {
    return(JOYState_TypeDef) JOY_NONE;
  }
  else if(!(tmp & JOY_SEL_PIN))
  {
    return(JOYState_TypeDef) JOY_SEL;
  }
  else if(!(tmp & JOY_DOWN_PIN))
  {
    return(JOYState_TypeDef) JOY_DOWN;
  } 
  else if(!(tmp & JOY_LEFT_PIN))
  {
    return(JOYState_TypeDef) JOY_LEFT;
  }
  else if(!(tmp & JOY_RIGHT_PIN))
  {
    return(JOYState_TypeDef) JOY_RIGHT;
  }
  else if(!(tmp & JOY_UP_PIN))
  {
    return(JOYState_TypeDef) JOY_UP;
  }
  else
  { 
    return(JOYState_TypeDef) JOY_NONE;
  }  
}

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* I2C Routines *********************************/
/**
  * @brief  Initializes I2C MSP.
  * @param  None
  * @retval None
  */
static void I2Cx_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
  EVAL_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();
  
  /* Configure I2C Tx as alternate function */
  GPIO_InitStruct.Pin = EVAL_I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = EVAL_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(EVAL_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure I2C Rx as alternate function */
  GPIO_InitStruct.Pin = EVAL_I2Cx_SDA_PIN;
  HAL_GPIO_Init(EVAL_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
  
  /*** Configure the I2C peripheral ***/ 
  /* Enable I2C clock */
  EVAL_I2Cx_CLK_ENABLE();
  
  /* Force the I2C peripheral clock reset */  
  EVAL_I2Cx_FORCE_RESET(); 
  
  /* Release the I2C peripheral clock reset */  
  EVAL_I2Cx_RELEASE_RESET(); 
  
  /* Enable and set I2Cx Interrupt to a lower priority */
  HAL_NVIC_SetPriority(EVAL_I2Cx_EV_IRQn, 0x05, 0);
  HAL_NVIC_EnableIRQ(EVAL_I2Cx_EV_IRQn);
  
  /* Enable and set I2Cx Interrupt to a lower priority */
  HAL_NVIC_SetPriority(EVAL_I2Cx_ER_IRQn, 0x05, 0);
  HAL_NVIC_EnableIRQ(EVAL_I2Cx_ER_IRQn);
}

/**
  * @brief  Initializes I2C HAL.
  * @param  None
  * @retval None
  */
static void I2Cx_Init(void)
{
  if(HAL_I2C_GetState(&heval_I2c) == HAL_I2C_STATE_RESET)
  {
    heval_I2c.Instance = I2C1;
    heval_I2c.Init.ClockSpeed      = I2C_SPEED;
    heval_I2c.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    heval_I2c.Init.OwnAddress1     = 0;
    heval_I2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    heval_I2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    heval_I2c.Init.OwnAddress2     = 0;
    heval_I2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    heval_I2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;  
    
    /* Init the I2C */
    I2Cx_MspInit();
    HAL_I2C_Init(&heval_I2c);    
  }
}

/**
  * @brief  Configures I2C Interrupt.
  * @param  None
  * @retval None
  */
static void I2Cx_ITConfig(void)
{
  static uint8_t I2C_IT_Enabled = 0;
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  if(I2C_IT_Enabled == 0)
  {
    I2C_IT_Enabled = 1;
    /* Enable the GPIO EXTI clock */
    __GPIOI_CLK_ENABLE();
    __SYSCFG_CLK_ENABLE();
    
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    
    /* Enable and set GPIO EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x0F, 0x0F);
    HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));
  }
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
static void I2Cx_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&heval_I2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 100); 

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  }
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
static uint8_t I2Cx_Read(uint8_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Mem_Read(&heval_I2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  }
  return Value;   
}

/**
  * @brief  Reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static HAL_StatusTypeDef I2Cx_ReadMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(&heval_I2c, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* I2C error occured */
    I2Cx_Error(Addr);
  }
  return status;    
}

/**
  * @brief  Writes a value in a register of the device through BUS in using DMA mode.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  pBuffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_WriteMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&heval_I2c, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(Addr);
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&heval_I2c, DevAddress, Trials, 1000));
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  Addr: I2C Address
  * @retval None
  */
static void I2Cx_Error(uint8_t Addr)
{
  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit(&heval_I2c);
  
  /* Re-Initiaize the I2C comunication bus */
  I2Cx_Init();
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK IOE ***********************************/

/**
  * @brief  Initializes IOE low level.
  * @param  None
  * @retval None
  */
void IOE_Init(void) 
{
  I2Cx_Init();
}

/**
  * @brief  Configures IOE low level interrupt.
  * @param  None
  * @retval None
  */
void IOE_ITConfig(void)
{
  I2Cx_ITConfig();
}

/**
  * @brief  IOE writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_Write(Addr, Reg, Value);
}

/**
  * @brief  IOE reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
  return I2Cx_Read(Addr, Reg);
}

/**
  * @brief  IOE reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
 return I2Cx_ReadMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  IOE writes multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */
void IOE_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  I2Cx_WriteMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  IOE delay 
  * @param  Delay: Delay in ms
  * @retval None
  */
void IOE_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/********************************* LINK AUDIO *********************************/

/**
  * @brief  Initializes Audio low level.
  * @param  None
  * @retval None
  */
void AUDIO_IO_Init(void) 
{
  I2Cx_Init();
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
  uint16_t tmp = Value;
  
  Value = ((uint16_t)(tmp >> 8) & 0x00FF);
  
  Value |= ((uint16_t)(tmp << 8)& 0xFF00);
  
  I2Cx_WriteMultiple(Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Data to be read
  */
uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg)
{
  uint16_t Read_Value = 0, tmp = 0;
  
  I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&Read_Value, 2); 
  
  tmp = ((uint16_t)(Read_Value >> 8) & 0x00FF);
  
  tmp |= ((uint16_t)(Read_Value << 8)& 0xFF00);
  
  Read_Value = tmp;
  
  return Read_Value;
}

/**
  * @brief  AUDIO Codec delay 
  * @param  Delay: Delay in ms
  * @retval None
  */
void AUDIO_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/********************************* LINK CAMERA ********************************/

/**
  * @brief  Initializes Camera low level.
  * @param  None
  * @retval None
  */
void CAMERA_IO_Init(void) 
{
  I2Cx_Init();
}

/**
  * @brief  Camera writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
void CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_Write(Addr, Reg, Value);
}

/**
  * @brief  Camera reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
uint8_t CAMERA_IO_Read(uint8_t Addr, uint8_t Reg)
{
  return I2Cx_Read(Addr, Reg);
}

/**
  * @brief  Camera delay 
  * @param  Delay: Delay in ms
  * @retval None
  */
void CAMERA_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/******************************** LINK I2C EEPROM *****************************/

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void EEPROM_IO_Init(void)
{
  I2Cx_Init();
}

/**
  * @brief  Write data to I2C EEPROM driver in using DMA channel.
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_WriteMultiple(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize));
}

/**
  * @brief  Read data from I2C EEPROM driver in using DMA channel.
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be read
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_ReadMultiple(DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize));
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (I2Cx_IsDeviceReady(DevAddress, Trials));
}
// Touch Screen Function----------------------------------
void TS_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Enable clock GPIOB,GPIOC*/
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	
	//Configure CS Pin out
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
		//Configure IRQ[I] & BUSY[I] 
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	/*Configure SPI2*/
	/*Configure MISO[I]*/
	GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
//	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
		/*Configure MOSI[O], SCK[O]*/
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
//	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	TP_CS(1);
	TP_DCLK(0);
	TP_DIN(0);
}
uint32_t TS_DetectTouch (void) 
{
	return !(TP_INT_IN);
}
uint16_t TS_Read_X(void){
/*	  int i;
    TS_WriteCMD(CHX);
   // while(TP_BUSY);
    TS_Delayus(40);
    i=TS_ReadAD();
    return i; */
	uint16_t i; 
  TP_CS(0); 
  TS_Delayus(40); 
  TS_WriteCMD(CHX); 
  TS_Delayus(40); 
  i=TS_ReadAD(); 
  TP_CS(1);
	TS_Delayus(40); 	
  return i;    
}
uint16_t TS_Read_Y(void){
	/*
	  int i;
    TS_WriteCMD(CHY);
    //while(TP_BUSY);
    TS_Delayus(40);
    i=TS_ReadAD();
    return i;  */
	uint16_t i; 
  TP_CS(0); 
  TS_Delayus(40); 
  TS_WriteCMD(CHY); 
  TS_Delayus(40); 
  i=TS_ReadAD(); 
  TP_CS(1); 
  return i;
}
void TS_Read_ADX(void){
	TP_CS(0); 
  TS_Delayus(40); 
  TS_WriteCMD(CHX); 
  TS_Delayus(40);
	TP_CS(1);
}
void TS_Read_ADY(void){
	TP_CS(0); 
  TS_Delayus(40); 
  TS_WriteCMD(CHY); 
  TS_Delayus(40);  
  TP_CS(1);
}
//////////////////////////////////Static Function///////////////////
void TS_WriteCMD (unsigned char cmd){
	  unsigned char buf;
    unsigned char i;
    TP_CS(1);
    TP_DIN(0);
    TP_DCLK(0);
    TP_CS(0);
    for(i=0;i<8;i++) 
    {
        buf=(cmd>>(7-i))&0x1;
        TP_DIN(buf);
        TS_Delayus(40);
        TP_DCLK(1);
        TS_Delayus(40);
        TP_DCLK(0);
    }
}
static uint16_t TS_ReadAD(void){
	  unsigned short buf=0,temp;
    unsigned char i;
    TP_DIN(0);
    TP_DCLK(1);
    for(i=0;i<12;i++) 
    {
        TS_Delayus(5);
        TP_DCLK(0);         
        TS_Delayus(5);   
        temp= (TP_DOUT) ? 1:0;
        buf|=(temp<<(11-i));
				TS_Delayus(5);
        TP_DCLK(1);
    }
    TP_CS(1);
    buf&=0x0fff;
    return(buf);
}
static void TS_Delayus( int k)
{
    int j;
    for(j=k << 8;j > 0;j--);
}
/***********************************************ADS7846 SPI2*****************************************************/
void TS2_Init(void){
   /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
	SpiHandle.Init.Mode							 = SPI_MODE_MASTER;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
	
	SPI_MspInit();
	HAL_SPI_Init(&SpiHandle);
	/*Set CS pin to active*/
	TP2_CS(1);
	
	  /*##-3- Configure the NVIC for SPI #########################################*/
  /* NVIC for SPI */
//  HAL_NVIC_SetPriority(SPIx_IRQn, 5, 0);
 // HAL_NVIC_EnableIRQ(SPIx_IRQn);
}
uint32_t TS2_DetectTouch(void)
{
  __IO uint8_t status = 1;
  if(HAL_GPIO_ReadPin(TP_IRQ_PORT, TP_IRQ_PIN) != GPIO_PIN_RESET) 
  {
    status = 0;
  }
  
  return status;
}
void TS2_EnableIT(void){
	GPIO_InitTypeDef GPIO_Init_Structure;
  
  /* Configure Interrupt mode for SD detection pin */ 
  GPIO_Init_Structure.Mode      = GPIO_MODE_IT_RISING_FALLING;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_FAST;
  GPIO_Init_Structure.Pin       = TP_IRQ_PIN;
  HAL_GPIO_Init(TP_IRQ_PORT, &GPIO_Init_Structure);
    
  /* NVIC configuration for SDIO interrupts */
  HAL_NVIC_SetPriority(TP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TP_IRQn);
}

static void SPI_MspInit(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	   /* Peripheral clock enable */
    __SPI2_CLK_ENABLE();
	
		/* GPIO clock enable*/
		__GPIOB_CLK_ENABLE();
		__GPIOC_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PB13     ------> SPI2_SCK
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		/*Configure IRQ[I], BUSY[I] ,CS[O] PIN*/
		
		GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}	
static void SPI_MspDeInit(void)
{
  /*##-1- Reset peripherals ##################################################*/
  SPIx_FORCE_RESET();
  SPIx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure SPI SCK as alternate function  */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
  /* Configure SPI MISO as alternate function  */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);
  /* Configure SPI MOSI as alternate function  */
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2);
  
  /*##-3- Disable the NVIC for SPI ###########################################*/
  HAL_NVIC_DisableIRQ(SPIx_IRQn);
}
uint16_t TS2_Read_X(void)
{
	uint16_t AdcValue;
	TP2_CS(0);
	SpiDelay(10);
	/*
		TSC2046 控制字（8Bit）
		Bit7   = S     起始位，必须是1
		Bit6:4 = A2-A0 模拟输入通道选择A2-A0; 共有6个通道。
		Bit3   = MODE  ADC位数选择，0 表示12Bit;1表示8Bit
		Bit2   = SER/DFR 模拟输入形式，  1表示单端输入；0表示差分输入
		Bit1:0 = PD1-PD0 掉电模式选择位
	*/
	SPI_ShiftByte(CHX);			/* 选择通道1, 测量X位置 */
	SpiDelay(10);
	/* 读ADC结果, 12位ADC值的高位先传，前12bit有效，最后4bit填0 */
	AdcValue = SPI_ShiftByte(0x00);		/* 发送的0x00可以为任意值，无意义 */
	AdcValue <<= 8;
	AdcValue += SPI_ShiftByte(0x00);		/* 获得12位的ADC采样值 */
	AdcValue >>= 3;						/* 右移3位，保留12位有效数字 */
	AdcValue &= 0xFFF;
	TP2_CS(1);	

	return (AdcValue);
}
uint16_t TS2_Read_Y(void)
{
	uint16_t AdcValue;
	TP2_CS(0);
	SpiDelay(10);
	/*
		TSC2046 控制字（8Bit）
		Bit7   = S     起始位，必须是1
		Bit6:4 = A2-A0 模拟输入通道选择A2-A0; 共有6个通道。
		Bit3   = MODE  ADC位数选择，0 表示12Bit;1表示8Bit
		Bit2   = SER/DFR 模拟输入形式，  1表示单端输入；0表示差分输入
		Bit1:0 = PD1-PD0 掉电模式选择位
	*/
	SPI_ShiftByte(CHY);			/* 选择通道1, 测量X位置 */
	SpiDelay(10);
	/* 读ADC结果, 12位ADC值的高位先传，前12bit有效，最后4bit填0 */
	AdcValue = SPI_ShiftByte(0x00);		/* 发送的0x00可以为任意值，无意义 */
	AdcValue <<= 8;
	AdcValue += SPI_ShiftByte(0x00);		/* 获得12位的ADC采样值 */
	AdcValue >>= 3;						/* 右移3位，保留12位有效数字 */
	AdcValue &= 0xFFF;
	TP2_CS(1);	

	return (AdcValue);
}
static uint8_t SPI_ShiftByte(uint8_t TxData)
{
	uint8_t RxData[1]={0};
	uint8_t TxByte[1]={0};
	
		TxByte[0] = TxData;
//		HAL_SPI_TransmitReceive_IT(&SpiHandle,TxByte, RxData,sizeof(TxByte));
		HAL_SPI_Transmit(&SpiHandle, TxByte,1,1);
		SpiDelay(10);
		HAL_SPI_Receive(&SpiHandle, RxData,1,1);
		SpiDelay(10);
	return RxData[0];
} 
static void TP2_CS(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_CS_PORT,TP_CS_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_CS_PORT,TP_CS_PIN,GPIO_PIN_RESET);
}
static void TP2_CLK(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_SPIx_SCK_GPIO_PORT,TP_SPIx_SCK_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_SPIx_SCK_GPIO_PORT,TP_SPIx_SCK_PIN,GPIO_PIN_RESET);
}
static void TP2_DIN(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_SPIx_MOSI_GPIO_PORT,TP_SPIx_MOSI_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_SPIx_MOSI_GPIO_PORT,TP_SPIx_MOSI_PIN,GPIO_PIN_RESET);
}
static void TP2_DOUT(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_SPIx_MISO_GPIO_PORT,TP_SPIx_MISO_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_SPIx_MISO_GPIO_PORT,TP_SPIx_MISO_PIN,GPIO_PIN_RESET);
}
static void SpiDelay(unsigned int DelayCnt)
{
 unsigned int i;
 for(i=0;i<DelayCnt;i++);
}


void TS3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Enable clock GPIOB,GPIOC*/
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	
	//Configure CS Pin out
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
		//Configure IRQ[I] & BUSY[I] 
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	/*Configure SPI2*/
	/*Configure MISO[I]*/
	GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
//	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
		/*Configure MOSI[O], SCK[O]*/
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
//	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	TP3_CS(1);
	TP3_CLK(0);
	TP3_DIN(0);
}

uint16_t TS3_Read_X(void){
	return TS3_Read_AD(CHX);
}
uint16_t TS3_Read_Y(void){
	return TS3_Read_AD(CHY);
}
uint32_t TS3_DetectTouch(void)
{
  __IO uint8_t status = 1;
  if(HAL_GPIO_ReadPin(TP_IRQ_PORT, TP_IRQ_PIN) != GPIO_PIN_RESET) 
  {
    status = 0;
  }
  
  return status;
}
static uint16_t TS3_Read_AD(uint8_t CMD)	  
{ 	 
	uint8_t i;
	uint8_t count=0; 	  
	uint16_t Num=0; 
//	T_DCLK_L;//先拉低时钟 	 
	TP3_CS(0); //选中ADS7843	 
	TS3_WriteCmd(CMD);//发送命令字
//	T_DCLK_L; 
	for(volatile int i=400;i>0;i--)__nop();
//	T_DCLK_H;//给1个时钟，清除BUSY  
//	__nop();__nop();
//	__nop();__nop(); 	    
//	T_DCLK_L; 	 
	for(uint8_t count=0;count<16;count++)  
	{ 				  
		Num<<=1; 
		TP3_CLK(0);//下降沿有效  
		__nop();__nop();
		__nop();__nop();
		TP3_CLK(1);
		__nop();__nop();
		__nop();__nop();
		if(HAL_GPIO_ReadPin(TP_SPIx_MOSI_GPIO_PORT,TP_SPIx_MOSI_PIN)){
			Num++; 	
		}			
	}  	
	Num>>=4;   //只有高12位有效.
	TP3_CS(1);//释放ADS7843	 
	return(Num);   
}


static void TS3_WriteCmd(uint8_t Cmd)    
{  
	uint8_t count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(Cmd&0x80)TP3_DOUT(1);  
		else TP3_DOUT(0);   
		Cmd<<=1;   
		TP3_CLK(0);//上升沿有效
		__nop();__nop();
		TP3_CLK(1); 
		__nop();__nop();
	} 			    
} 
static void TP3_CS(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_CS_PORT,TP_CS_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_CS_PORT,TP_CS_PIN,GPIO_PIN_RESET);
}
static void TP3_CLK(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_SPIx_SCK_GPIO_PORT,TP_SPIx_SCK_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_SPIx_SCK_GPIO_PORT,TP_SPIx_SCK_PIN,GPIO_PIN_RESET);
}
static void TP3_DIN(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_SPIx_MOSI_GPIO_PORT,TP_SPIx_MOSI_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_SPIx_MOSI_GPIO_PORT,TP_SPIx_MOSI_PIN,GPIO_PIN_RESET);
}
static void TP3_DOUT(uint8_t State){
	if(State)
	HAL_GPIO_WritePin(TP_SPIx_MISO_GPIO_PORT,TP_SPIx_MISO_PIN,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(TP_SPIx_MISO_GPIO_PORT,TP_SPIx_MISO_PIN,GPIO_PIN_RESET);
}


//******************************************LINK FMC********************************************************8
/*************************** FSMC Routines ************************************/
/**
  * @brief  Initializes FSMC_BANK3 MSP.
  * @param  None
  * @retval None
  */
static void FMC_BANK1_MspInit(void)
{
 GPIO_InitTypeDef GPIO_Init_Structure;
    
  /* Enable FSMC clock */
  __FMC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  
  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;
  
  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |GPIO_PIN_7| GPIO_PIN_8     |\
                              GPIO_PIN_9 | GPIO_PIN_10 |GPIO_PIN_14 | GPIO_PIN_15;
   
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_3| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | \
															GPIO_PIN_11 | GPIO_PIN_12 |GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);
  
  /* GPIOF configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
                              GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);
  
  /* GPIOG configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
                              GPIO_PIN_5 |GPIO_PIN_9;
  
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);  
	 /* Blacklight configuration PB0 */
  GPIO_Init_Structure.Pin = GPIO_PIN_0;
  GPIO_Init_Structure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init_Structure.Pull=GPIO_NOPULL;
	GPIO_Init_Structure.Speed=GPIO_SPEED_FAST;
	GPIO_Init_Structure.Alternate=GPIO_AF0_MCO;
	HAL_GPIO_Init(GPIOB,&GPIO_Init_Structure);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);

}

/**
  * @brief  Initializes LCD IO.
  * @param  None
  * @retval None
  */
static void FMC_BANK1_Init(void) 
{  
  SRAM_HandleTypeDef hsram;
  FMC_NORSRAM_TimingTypeDef SRAM_Timing;
  
  /*** Configure the SRAM Bank 1 ***/  
  /* Configure IPs */
  hsram.Instance  = FMC_NORSRAM_DEVICE;
  hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

  SRAM_Timing.AddressSetupTime      = 1;
  SRAM_Timing.AddressHoldTime       = 0;
  SRAM_Timing.DataSetupTime         = 9;
  SRAM_Timing.BusTurnAroundDuration = 0;
  SRAM_Timing.CLKDivision           = 0;
  SRAM_Timing.DataLatency           = 0;
  SRAM_Timing.AccessMode            = FMC_ACCESS_MODE_A;
  
  hsram.Init.NSBank             = FMC_NORSRAM_BANK1; //chon Bank 1
  hsram.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
  hsram.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
	hsram.Init.AsynchronousWait 	= FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram.Init.WrapMode           = FMC_WRAP_MODE_DISABLE;
  hsram.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
  hsram.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
  hsram.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
  hsram.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
  hsram.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;

 
  /* Initialize the SRAM controller */
  FMC_BANK1_MspInit();
  HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing);   
}

/**
  * @brief  Writes register value.
  * @param  Data: Data to be written 
  * @retval None
  */
static void FMC_BANK1_WriteData(uint16_t Data) 
{
  /* Write 16-bit Reg */
 // FMC_BANK3->RAM = Data;
	LCD_DATA=Data;
}

/**
  * @brief  Writes register address.
  * @param  Reg: Register to be written
  * @retval None
  */
static void FMC_BANK1_WriteReg(uint8_t Reg) 
{
  /* Write 16-bit Index, then write register */
//  FMC_BANK3->REG = Reg;
	LCD_CMD=Reg;
}

/**
  * @brief  Reads register value.
  * @param  None
  * @retval Read value
  */
static uint16_t FMC_BANK1_ReadData(void) 
{
 // return FMC_BANK3->RAM;
	return LCD_DATA;
}
//****************************************************LINK LCD**************************************************
/********************************* LINK LCD ***********************************/

/**
  * @brief  Initializes LCD low level.
  * @param  None
  * @retval None
  */
void LCD_IO_Init(void) 
{
  if(Is_LCD_IO_Initialized == 0)
  {
    Is_LCD_IO_Initialized = 1; 
    FMC_BANK1_Init();
  }
}

/**
  * @brief  Writes data on LCD data register.
  * @param  Data: Data to be written
  * @retval None
  */
void LCD_IO_WriteData(uint16_t Data) 
{
  /* Write 16-bit Reg */
  FMC_BANK1_WriteData(Data);
}

/**
  * @brief  Writes register on LCD register.
  * @param  Reg: Register to be written
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t Reg) 
{
  /* Write 16-bit Index, then Write Reg */
  FMC_BANK1_WriteReg(Reg);
}

/**
  * @brief  Reads data from LCD data register.
  * @param  None
  * @retval Read data.
  */
uint16_t LCD_IO_ReadData(void) 
{
  return FMC_BANK1_ReadData();
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
