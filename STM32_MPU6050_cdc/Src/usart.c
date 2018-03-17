/*
 * Open-BLDC - Open BrushLess DC Motor Controller
 * Copyright (C) 2009-2010 by Piotr Esden-Tempski <piotr@esden.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "usb_lib.h"
#include "usb_pwr.h"
#include "usart.h"
#include "ringbuffer.h"

#define USART__BAUD  9600

static volatile s16 data_buf;

extern ringbuffer_t *pRingbuffer;
//extern  __IO uint32_t bDeviceState;

void usart_init(void)
{
   NVIC_InitTypeDef nvic;
   GPIO_InitTypeDef gpio;
   USART_InitTypeDef usart;

   /* enable clock for USART1 peripherial */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 |
         RCC_APB2Periph_GPIOB |
         RCC_APB2Periph_AFIO, ENABLE);

   /* Enable the USART1 interrupts */
   nvic.NVIC_IRQChannel = USART1_IRQn;
   nvic.NVIC_IRQChannelPreemptionPriority = 3;
   nvic.NVIC_IRQChannelSubPriority = 0;
   nvic.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&nvic);

   /* enable USART1 pin software remapping */
   GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

   /* GPIOB: USART1 Tx push-pull */
   //GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
   gpio.GPIO_Pin = GPIO_Pin_6;
   gpio.GPIO_Mode = GPIO_Mode_AF_PP;
   gpio.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &gpio);

   /* GPIOB: USART1 Rx pin as floating input */
   gpio.GPIO_Pin = GPIO_Pin_7;
   gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB, &gpio);

   /* Initialize the usart subsystem */
   usart.USART_BaudRate = USART__BAUD;
   usart.USART_WordLength = USART_WordLength_8b;
   usart.USART_StopBits = USART_StopBits_1;
   usart.USART_Parity = USART_Parity_No;
   usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   /* Configure USART1 */
   USART_Init(USART1, &usart);

   /* Enable USART1 Receive and Transmit interrupts */
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

   /* Enable the USART1 */
   USART_Cmd(USART1, ENABLE);
}

void usart_enable_send(void)
{
   USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void usart_disable_send(void)
{
   USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
}

void USART1_IRQHandler(void)
{
   char dispbuffer[4];
   u8 tmp;

   /* input (RX) handler */
   if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
      data_buf = (s16)USART_ReceiveData(USART1);
      sprintf(dispbuffer, "%c", data_buf);
      tmp = strlen(dispbuffer);
      ringbuffer_write(pRingbuffer, (u8*)dispbuffer, tmp);
      if ((bDeviceState == CONFIGURED) && (GetEPTxStatus(ENDP1) == EP_TX_NAK))
      {
         // not sending anything, so just call EP1_IN_Callback()..
         extern void EP1_IN_Callback (void);
         EP1_IN_Callback();
      }
   }

   /* output (TX) handler */
#if 0   // not used for now..
   if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
      if ((data_buf = gpc_pickup_byte()) >= 0) {
         USART_SendData(USART1, (uint16_t)data_buf);
      } else {
         usart_disable_send();
      }
   }
#endif
}



/*
 * http://electronics.stackexchange.com/questions/248967/how-do-i-use-both-usart1-and-usart2-together-in-stm32f103
 */
#if 0
void GPIO_CONFIG(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   // enable uart 1 preph clock mode
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

   // enable uart 2 preph clock mode
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

   /* Configure USART1 Tx (PA.09) and UARTT2 Tx (PA.02) as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Configure USART1 Rx (PA.10) and USART2 Rx (PA.03) as input floating */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
}
#endif





