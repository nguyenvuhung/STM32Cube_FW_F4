/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for main.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __K_LCD_BRIGHTNESS_H
#define __K_LCD_BRIGHTNESS_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
void PWM_Init(void);
void LCD_Brightness(uint8_t Duty);

#endif /* __K_LCD_BRIGHTNESS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
