/**
 ******************************************************************************
 * @file    usb_endp.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Endpoint routines
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

#include "ringbuffer.h"

extern ringbuffer_t *pRingbuffer;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL 10
uint8_t ep3_packet_buffer[VIRTUAL_COM_PORT_DATA_SIZE];
uint8_t ep1_packet_buffer[VIRTUAL_COM_PORT_DATA_SIZE];


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : EP1_IN_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void EP1_IN_Callback (void)
{
   // Host asking for data
   u32 rd;

   /* wait until the data transmission is finished */
    while (GetEPTxStatus(ENDP1) == EP_TX_VALID); //0x30

    rd = ringbuffer_read(pRingbuffer, ep1_packet_buffer, sizeof(ep1_packet_buffer));
    if (rd > 0)
    {
       USB_SIL_Write(ENDP1, ep1_packet_buffer, rd);
       SetEPTxValid(ENDP1);
    }
    else
    {
       SetEPTxStatus(ENDP1, EP_TX_NAK);
    }
}

/*******************************************************************************
 * Function Name  : EP3_OUT_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void EP3_OUT_Callback(void)
{
   // Host has send data
   uint32_t packet_size;

   packet_size = (uint8_t)(USB_SIL_Read(EP3_OUT, ep3_packet_buffer));
   SetEPRxValid(ENDP3);
}


/*******************************************************************************
 * Function Name  : SOF_Callback / INTR_SOFINTR_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void SOF_Callback(void)
{
   static uint32_t FrameCount = 0;

   if(bDeviceState == CONFIGURED)
   {
      if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
      {
         /* Reset the frame counter */
         FrameCount = 0;
         //Handle_USBAsynchXfer();
      }
   }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

