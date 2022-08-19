/*
 * Global_def.h
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#ifndef INC_GLOBAL_DEF_H_
#define INC_GLOBAL_DEF_H_

/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
/* @@@@@@@@@@@@@ Select the GPS module @@@@@@@@@@@@@ */
/* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
#define UBLOX_NEO_8
//#define NOVATEL_OEM_719

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "inc/SPI_driver.h"
#include "inc/SCI_driver.h"
#include "inc/Decode.h"

#if defined(NOVATEL_OEM_719)
#include "inc/Novatel_OEM719.h"
#elif defined(UBLOX_NEO_8)
#include "inc/Ublox_8_Receiver.h"
#endif

#include "inc/SDCARD_driver.h"
#include "inc/SD_Queue.h"
#include "sd_card/include/ff.h"            /* FatFs declarations */
#include "sd_card/include/diskio.h"        /* Include file for user provided disk functions */
#include "sd_card/include/integer.h"

#include <string.h>
#include "stdlib.h"
#include "math.h"
#include "stdio.h"

//////////////////////////////////////////////////////////////////////////////////////////////Math
#define DegToRadian        0.0174532925199433
#define RadianToDeg        57.2957795130823
//////////////////////////////////////////////////////////////////////////////////////////////Global DEFINE
#define  DEF_SETFLAG(a)     ( a = SET )
#define  DEF_RESETFLAG(a)   ( a = RESET )
typedef enum {RESET = 0, SET = !RESET} FlagStatus;

//////////////////////////////////////////////////////////////////////////////////////////////LED
#define LED1_ON (GpioDataRegs.GPASET.bit.GPIO10 = 1)
#define LED1_OFF (GpioDataRegs.GPACLEAR.bit.GPIO10 = 1)
#define LED1_TOGGLE (GpioDataRegs.GPATOGGLE.bit.GPIO10 = 1)
#define LED2_ON (GpioDataRegs.GPASET.bit.GPIO11 = 1)
#define LED2_OFF (GpioDataRegs.GPACLEAR.bit.GPIO11 = 1)
#define LED2_TOGGLE (GpioDataRegs.GPATOGGLE.bit.GPIO11 = 1)

//////////////////////////////////////////////////////////////////////////////////////////////Timer
#define TIMCOUNT_1HZ             0x1869F //99999
#define TIMCOUNT_10HZ            0x270F //9999
#define TIMCOUNT_100HZ           0x3E7  //999
#define TIMCOUNT_250HZ           0xF9  //249
#define TIMCOUNT_1000HZ          0x63   //99

#define TIM0COUNT               TIMCOUNT_1HZ
#define TIM1COUNT               TIMCOUNT_250HZ

#define Get_tim0                Small_scale_timer0 = ReadCpuTimer0Counter()
#define Get_tim1                Small_scale_timer1 = ReadCpuTimer0Counter()
#define DEF_T0_BEGIN            T0CNT0 = (TIM0COUNT - Small_scale_timer0) + (TIM0COUNT+1)*(CpuTimer0.InterruptCount)
#define DEF_T0_END              T0CNT1 = (TIM0COUNT - Small_scale_timer0) + (TIM0COUNT+1)*(CpuTimer0.InterruptCount)
//////Timer0 이외의 Timer에 대한 Timetag는 거의 쓸일이 없을 것임!!
#define DEF_T1_BEGIN            T1CNT0 = (TIM1COUNT - Small_scale_timer1) + (TIM1COUNT+1)*(CpuTimer1.InterruptCount)
#define DEF_T1_END              T1CNT1 = (TIM1COUNT - Small_scale_timer1) + (TIM1COUNT+1)*(CpuTimer1.InterruptCount)

//////////////////////////////////////////////////////////////////////////////////////////////100Hz_IMU

//////////////////////////////////////////////////////////////////////////////////////////////


#endif
