/*
 *  File:     main.c
 *  Purpose:  RC Transmitter with STM32F103C8T6 and NRF24L01+
 *
 *  Date:     05 July 2013
 *  Info:     If __NO_SYSTEM_INIT is defined in the Build options,
 *            the startup code will not branch to SystemInit()
 *            and the function can be removed
 ************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stm32f10x.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_init.h"

#include "MPU6050.h"
#include "delay.h"
#include "st7735.h"
#include "garmin-digits.h"

#include "ringbuffer.h"
#include "usart.h"

#define USE_LCD      0  // 1 if to use ST7735 LCD, 0 to comment out..

int gyro_x = 0, gyro_y = 0, gyro_z = 0;
long acc_x = 0, acc_y = 0, acc_z = 0, acc_total_vector = 0;
long gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;
float angle_pitch = 0.0, angle_roll = 0.0;
int angle_pitch_buffer = 0, angle_roll_buffer = 0;
bool set_gyro_angles = FALSE;
float angle_roll_acc = 0.0, angle_pitch_acc = 0.0;
float angle_pitch_output = 0.0, angle_roll_output = 0.0;

extern __IO uint32_t syscnt;

u32 mpu_buffer[128];
ringbuffer_t *pRingbuffer;


#ifndef __NO_SYSTEM_INIT
//sg: already defined in system_stm32f10x.c
void SystemInit (void) __attribute__((weak));

void SystemInit()
{}
#endif

/*
 * Delay must not be > 0x80000000L..
 */
int IsDelayOver(uint32_t start, uint32_t delay)
{
   uint32_t cur = syscnt;

   if ((cur - (start + delay)) < 0x80000000L)
      return 1; // delay is over
   else
      return -1; // not yet..
}


void DelayMSec(uint32_t msec)
{
   uint32_t curtick = syscnt;

   while(IsDelayOver(curtick, msec) < 0); // wait ms
}

void NVIC_Configuration(void)
{
   //NVIC_InitTypeDef NVIC_InitStructure;

   /* Set the Vector Table base location at 0x08000000 */
   //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); // already done in SystemInit()

   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

   //NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}


void Initialize_LCD(void)
{
#if (USE_LCD==1)
   ST7735_Init();
   ST7735_AddrSet(0,0,159,127);
   ST7735_Clear(0x0000);
   ST7735_Orientation(scr_normal);
#endif
}

/*
 * Call from main() to test usb cdc implementation..
 * continously send "Test String #" to usb.
 */
void main_usbvcp_loop(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   uint8_t cnt = 0;
   int tmp;
   u32 starttime;
   char dispbuffer[64];
   u32 loopcnt = 0;

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   /* Configure PC13 */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   while(1)
   {
      // TESTING of VCP..
      starttime = syscnt;

      // write readings to ringbuffer for sending via usb cdc to a PC..
      sprintf(dispbuffer, "Test String %lu\n", loopcnt++);
      cnt = strlen(dispbuffer);
      if (ringbuffer_available(pRingbuffer) > cnt) // write only if enough storage available
      {
         tmp = ringbuffer_write(pRingbuffer, (u8*)dispbuffer, cnt);
         if ((bDeviceState == CONFIGURED) && (GetEPTxStatus(ENDP1) == EP_TX_NAK))
         {
            // not sending anything, so just call EP1_IN_Callback()..
            extern void EP1_IN_Callback (void);
            EP1_IN_Callback();
         }
      }

      GPIO_SetBits(GPIOC, GPIO_Pin_13);
      while(IsDelayOver(starttime, 40) < 0);
      GPIO_ResetBits(GPIOC, GPIO_Pin_13);
   }
}


/*
 * MPU6050 main loop.
 */
void main_mpu_loop(void)
{
   uint8_t cnt = 0;
   int i, tmp;
   u32 starttime;
   char dispbuffer[64];
   s16 mpu_val[6];

   MPU6050_I2C_Init();
   if (MPU6050_TestConnection())
   {
      MPU6050_Initialize();

      // compute the offsets by totaling 2000 readings, and getting their averages..
      gyro_x_cal = gyro_y_cal = gyro_z_cal = 0;
      for(i=0; i<2000; i++)
      {
         starttime = syscnt;
         MPU6050_GetRawAccelGyro(mpu_val);
         gyro_x_cal += mpu_val[3];
         gyro_y_cal += mpu_val[4];
         gyro_z_cal += mpu_val[5];

         // delay to simulate 250Hz
         while(IsDelayOver(starttime, 4) < 0);
      }
      gyro_x_cal /= 2000;
      gyro_y_cal /= 2000;
      gyro_z_cal /= 2000;

      while(1)
      {
         starttime = syscnt;
         MPU6050_GetRawAccelGyro(mpu_val);            //Read the raw acc and gyro data from the MPU-6050

         acc_x = mpu_val[0];
         acc_y = mpu_val[1];
         acc_z = mpu_val[2];
         gyro_x = mpu_val[3] - gyro_x_cal;            //Subtract the offset calibration value from the raw gyro_x value
         gyro_y = mpu_val[4] - gyro_y_cal;            //Subtract the offset calibration value from the raw gyro_y value
         gyro_z = mpu_val[5] - gyro_z_cal;            //Subtract the offset calibration value from the raw gyro_z value
#if 1
         // write readings to ringbuffer for sending via usb cdc to a PC..
         sprintf(dispbuffer, "%d %d %d %d %d %d\n",
               /*mpu_val[0], mpu_val[1], mpu_val[2], mpu_val[3], mpu_val[4], mpu_val[5]*/
               acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
         );
         cnt = strlen(dispbuffer);
#else
         // format data for SerialPort-RealTime-Data-Plotter
         cnt = 0;
         dispbuffer[cnt++] = (u8)(acc_x >> 8);
         dispbuffer[cnt++] = 0x20; // space character
         dispbuffer[cnt++] = (u8)(acc_x);
         dispbuffer[cnt++] = 0x20; // space character

         dispbuffer[cnt++] = (u8)(acc_y >> 8);
         dispbuffer[cnt++] = 0x20; // space character
         dispbuffer[cnt++] = (u8)(acc_y);
         dispbuffer[cnt++] = 0x20; // space character

         dispbuffer[cnt++] = (u8)(acc_z >> 8);
         dispbuffer[cnt++] = 0x20; // space character
         dispbuffer[cnt++] = (u8)(acc_z);
         dispbuffer[cnt++] = '\n'; // terminate with LF character
#endif
         if (ringbuffer_available(pRingbuffer) > cnt) // write only if enough storage available
         {
            tmp = ringbuffer_write(pRingbuffer, (u8*)dispbuffer, cnt);
            if ((bDeviceState == CONFIGURED) && (GetEPTxStatus(ENDP1) == EP_TX_NAK))
            {
               // not sending anything, so just call EP1_IN_Callback()..
               extern void EP1_IN_Callback (void);
               EP1_IN_Callback();
            }
         }

         //Gyro angle calculations
         //0.0000611 = 1 / (250Hz / 65.5)
         angle_pitch += gyro_x * 0.0000611;           //Calculate the traveled pitch angle and add this to the angle_pitch variable
         angle_roll += gyro_y * 0.0000611;            //Calculate the traveled roll angle and add this to the angle_roll variable

         //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
         angle_pitch += angle_roll * sin(gyro_z * 0.000001066);      //If the IMU has yawed transfer the roll angle to the pitch angel
         angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);      //If the IMU has yawed transfer the pitch angle to the roll angel

         //Accelerometer angle calculations
         acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));  //Calculate the total accelerometer vector
         //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
         angle_pitch_acc = asin((float)acc_y/acc_total_vector) * 57.296;       //Calculate the pitch angle
         angle_roll_acc = asin((float)acc_x/acc_total_vector) * -57.296;       //Calculate the roll angle

         //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
         angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
         angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

         if (set_gyro_angles)                                                 //If the IMU is already started
         {
            angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
            angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
         }
         else                                                                //At first start
         {
            angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
            angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
            set_gyro_angles = TRUE;                                            //Set the IMU started flag
         }

         //To dampen the pitch and roll angles a complementary filter is used
         angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
         angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

         // since the values are x10, to see angle, divide by 10 will give angle to 1 decimal place..
         angle_pitch_buffer = angle_pitch_output * 10;
         angle_roll_buffer = angle_roll_output * 10;

#if (USE_LCD==1)
         //write_LCD();                                                       //Write the roll and pitch values to the LCD display
         sprintf(dispbuffer, "%d.%d", angle_pitch_buffer/10, abs(angle_pitch_buffer%10));
         ST7735_PutStr5x7(0,10,dispbuffer,RGB565(0,255,0));
         sprintf(dispbuffer, "%d.%d", angle_roll_buffer/10, abs(angle_roll_buffer%10));
         ST7735_PutStr5x7(0,20,dispbuffer,RGB565(0,0,255));
#endif
         // delay to simulate 250Hz loop time
         while(IsDelayOver(starttime, 4) < 0);
      }
   }
   else
   {

   }
}



/*
 * Call from main() to read from ublox neo-6m GPS..
 * continously send data read to usb.
 */
void main_gps_loop(void)
{
   int tmp;
   u32 starttime;

   usart_init();

   while(1)
   {
      starttime = syscnt;

      // do nothing.. all done in usart ISR

      while(IsDelayOver(starttime, 40) < 0);
   }
}



/*********************************************************************
 *
 *  main()
 *
 *********************************************************************/
int main(void)
{
   /******************************************************************
    *
    *
    ******************************************************************/
   char dispbuffer[32];
   u32 starttime;
   u8 run_gps, run_vcp, run_mpu;

   //DBGMCU_Config(DBGMCU_IWDG_STOP,ENABLE);
   //DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);

#if 1
   // Setup USB..
   Set_System();
   CAN_DeInit(CAN1);
   CAN_DBGFreeze(CAN1,DISABLE);

   Set_USBClock();
   USB_Interrupts_Config();
   USB_Init();
#endif
   NVIC_Configuration();

   if (SysTick_Config(SystemCoreClock / 1000)) // 1ms interval
   {
      /* Capture error */
      while (1);
   }

   pRingbuffer = (ringbuffer_t *)mpu_buffer;
   ringbuffer_init(pRingbuffer, sizeof(mpu_buffer));

   //Delay_ms(50);
   starttime = syscnt;
   while(IsDelayOver(starttime, 100) < 0);

   Initialize_LCD();
#if (USE_LCD==1)
   strcpy(dispbuffer, "INITIALISED");
   ST7735_PutStr5x7(0,50,dispbuffer,RGB565(0,255,0));
#endif

   // select which one to run.. only 1 can be active.
   run_gps = 0;
   run_vcp = 1;
   run_mpu = 0;

   if (run_vcp) main_usbvcp_loop();
   if (run_gps) main_gps_loop();
   if (run_mpu) main_mpu_loop();

   while(1)
   {
   }

   return 0;
}




#if 0
/*
 * This code shows an easy way to smooth readings from a sensor subject to
 * high frequency noise.
 * It uses a low pass filter on a circular buffer.
 * This circular buffer always contains the last BUFFER_SIZE-1 readings from
 * the sensor.
 * The new reading is then added to this buffer, from which wecompute the
 * mean value by simply dividing the sum of the readings in the buffer by the
 * number of readings in the buffer.
 */

int indexBuffer;
float circularBuffer[BUFFER_SIZE];
float sensorDataCircularSum;
int BUFFER_SIZE; // Number of samples you want to filter on.
float filteredOutput;
float sensorRawData; // typically the value you read from your sensor in your loop() function

void smoothSensorReadings(void)
{
   // We remove the oldest value from the buffer
   sensorDataCircularSum= sensorDataCircularSum - circularBuffer[indexBuffer];
   // The new input from the sensor is placed in the buffer
   circularBuffer[indexBuffer]=sensorRawData;
   // It is also added to the total sum of the last  BUFFER_SIZE readings
   // This method avoids to sum all the elements every time this function is called.
   sensorDataCircularSum+=sensorRawData;
   // We increment the cursor
   indexBuffer++;

   if (indexBuffer>=BUFFER_SIZE) indexBuffer=0;// We test if we arrived to the end
   //of the buffer, in which case we start again from index 0
   filteredOutput =(SensorDataCircularSum/BUFFER_SIZE); // The output is the the mean
   //value of the circular buffer.
}

#endif




