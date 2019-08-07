#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO�ڲ���
#include  "MK60_uart.h"     //����
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //�͹��Ķ�ʱ��(��ʱ)
#include  "MK60_i2c.h"      //I2C
#include  "MK60_spi.h"      //SPI
#include  "MK60_pit.h"      //PIT
#include  "MK60_FLASH.h"    //FLASH
#include  "MK60_dma.h"
//#include  "MK60_sdhc.h"
#include  "MK60_dac.h"
#include  "FIRE_SCCB.h"

#include   "MK60_adc.h"
#include   "MK60_FTM.h"
#include   "UART.h"
#include   "FIRE_camera.h"
#include   "SYN6288.h"
#include   "lcd.h"
#include  "FIRE_LED.H"          //LED
#include  "FIRE_KEY.H"          //KEY
#include  "FIRE_MMA7455.h"      //������ٶ�MMA7455
#include  "FIRE_NRF24L0.h"      //����ģ��NRF24L01+

#include  "stdbool.h"
#include  "control.h"
#include  "IIC.h"
#include  "Fuzzy_PID.h"
#include  "ff.h"                //FatFs
#include  "ffconf.h"
#include  "FML_DiskIO.h"
#include  "HAL_SDHC.h"
#include  "diskio.h"
#include <stdio.h>
#include  "io.h"
#include  "camera.h"
#include  "Recognition.h"
#include  "TestFunction.h"

#include  "sora_k60_ftm.h"


#endif  //__INCLUDE_H__
