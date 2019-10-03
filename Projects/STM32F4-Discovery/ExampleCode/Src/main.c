/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.5
  * @date    29-January-2016
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "main.h"

/* Example code for examine area - START */

/* Example code for examine area - END */

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Example code for examine area - START */
  /* Examine 1: copy 16 byte content of 0x20000100 to 0x20000110 */
uint32_t *B0_Buffer = (volatile uint32_t*)0x20000100;
uint32_t *B1_Buffer = (volatile uint32_t*)0x20000110;
uint32_t *Temp_Buffer;
uint8_t loop;

/* Init value for address 0x2000100, length 16 bytes */
Temp_Buffer = B0_Buffer;
for (loop = 0; loop < 16/4; loop++)
  {
    *Temp_Buffer++ = ((loop + 1) << 24) | ((loop + 2) << 16)
                    | ((loop + 3) << 8) | ((loop + 4) << 0);
  }
/* Coping 16 bytes start from 0x2000000 to 0x2000010 */
for(loop = 0; loop < 16/4; loop++)
  {
    *B1_Buffer++ = *B0_Buffer++;
  }

/* Examine 2: copy 16 byte content of 0x20000100 to 0x20000108
 *  - need manage overlap between 2 buffer
 *  [0x20000100] -> [0x20000108]
 *  [0x20000104] -> [0x2000010C]
 *  [0x20000108] -> [0x20000110]
 *  [0x2000010C] -> [0x20000114]
 */
uint32_t *A0_Buffer = (volatile uint32_t*)0x20000100;
uint32_t *A1_Buffer = (volatile uint32_t*)0x20000108;
uint32_t Temp_Array[4]; /*  store 16 byte value from address 0x20000100 */
uint32_t *A0_BufTemp;
uint8_t loop2;

/* Init value for address 0x2000100, length 16 bytes */
A0_BufTemp = B0_Buffer;
for (loop2 = 0; loop2 < 16/4; loop2++)
  {
    *A0_BufTemp++ = ((loop2 + 1) << 24) | ((loop2 + 2) << 16)
                    | ((loop2 + 3) << 8) | ((loop2 + 4) << 0);
  }

/*  store 16 byte value from address 0x20000100 */
for(loop2 = 0; loop2 < 16/4; loop2++)
  {
    Temp_Array[loop2] = *A0_Buffer++;
  }

/* copy Temp_Array[] to address 0x20000108 */
for(loop2 = 0; loop2 < 16/4; loop2++)
  {
    *A1_Buffer++ = Temp_Array[loop2];
  }


/* Examine 3: Use unalignment in C */
/* copy data buffer C0 start at address 0x20000100, length 14 bytes to buffer C1 at 0x20000112 */
 uint8_t *C0_Buffer = (volatile  uint8_t*)0x20000100;
 uint8_t *C1_Buffer = (volatile  uint8_t*)0x20000112;
uint32_t *C0_BufTemp;
uint8_t loop3;

for(loop3 = 0; loop3 < 14; loop3++)
  {
    *C1_Buffer++ = *C0_Buffer++;
  }

/* unalgined */
uint8_t tmp = 0x99;

  uint32_t* pMyPointer = (uint32_t *)(&tmp);;
  memset(pMyPointer, 0, sizeof(uint32_t));

trace_printf("&tmp = %p\n",&tmp);
trace_printf("tmp = %x\n",*((uint32_t *)(&tmp)));
trace_printf("pMyPointer = %p\n",pMyPointer);
trace_printf("*pMyPointer = %x\n",*pMyPointer);
  /* Example code for examine area - END */
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
