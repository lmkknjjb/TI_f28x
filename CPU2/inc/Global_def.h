/*
 * Global_def.h
 *
 *  Created on: 2018. 12. 7.
 *      Author: LEE Juhwan
 */

#ifndef INC_GLOBAL_DEF_H_
#define INC_GLOBAL_DEF_H_

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
//#include "inc/SPI_driver.h"

//////////////////////////////////////////////////////////////////////////////////////////////Grobal DEFINE
#define  DEF_SETFLAG(a)     ( a = SET )
#define  DEF_RESETFLAG(a)   ( a = RESET )
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

//////////////////////////////////////////////////////////////////////////////////////////////Timer
#define TIMCOUNT_01HZ           0x1869F //99999
#define TIMCOUNT_1HZ            0x270F //9999
#define TIMCOUNT_10HZ           0x3E7  //999
#define TIMCOUNT_100HZ          0x63   //99
#define TIMCOUNT_1000HZ         0x9  //9
#define TIM0COUNT               TIMCOUNT_100HZ
#define TIM1COUNT               TIMCOUNT_1HZ

Uint32 Tim0Flag = 0;
Uint32 Tim1Flag = 0;
Uint32 T0CNT0 = 0;
Uint32 T0CNT1 = 0;
Uint32 T1CNT0 = 0;
Uint32 T1CNT1 = 0;

#define DEF_T0_BEGIN          ( T0CNT0 = CpuTimer0.RegsAddr->TIM.all )
#define DEF_T0_END            ( T0CNT1 = (CpuTimer0.RegsAddr->TIM.all)+ (TIM0COUNT+1)*(Tim0Flag) )
//////Timer0 이외의 Timer에 대한 Timetag는 거의 쓸일이 없을 것임!!
#define DEF_T1_BEGIN          ( T1CNT0 = CpuTimer1.RegsAddr->TIM.all )
#define DEF_T1_END            ( T1CNT1 = CpuTimer1.RegsAddr->TIM.all + (TIM1COUNT+1)*(Tim1Flag) )


//////////////////////////////////////////////////////////////////////////////////////////////SD_카드 저장 GSRAM 변수
#pragma DATA_SECTION(rdata, "IMU_data");
#pragma DATA_SECTION(IMU_TIME, "IMU_time");

Uint16 rdata[13];
Uint32 IMU_TIME;       // IMU TIME tag
//////////////////////////////////////////////////////////////////////////////////////////////


#endif
