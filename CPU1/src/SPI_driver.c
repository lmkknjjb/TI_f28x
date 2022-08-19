/*
 * SPI_driver.c
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "inc/Global_def.h"

int16 IMUpacket[REC1_SZ] = {0xFEFF, 0x05FD, 0x0011, }; //Header:0xFFFEFD, ID:5, Size:17(0x11) Little indian

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////      IMU 통신관련영역         //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void TX_BufferAddress_ADIS16448(Uint16* pBuff){
    //IMU REGISTER
    pBuff[CMD] = GLOB_CMD | ADIS16448_READ;
    pBuff[DIAG_STAT] = 0x00;
    pBuff[XGYRO] = 0x00;
    pBuff[YGYRO] = 0x00;
    pBuff[ZGYRO] = 0x00;
    pBuff[XACCL] = 0x00;
    pBuff[YACCL] = 0x00;
    pBuff[ZACCL] = 0x00;
    pBuff[XMAGN] = 0x00;
    pBuff[YMAGN] = 0x00;
    pBuff[ZMAGN] = 0x00;
    pBuff[BARO] = 0x00;
    pBuff[TEMP] = 0x00;
}

void UpdateIMUPacket(Uint16* IMUdata, Uint32* IMUTime){

    Uint16 idx = 3; // Offset of header,id,size
    memcpy(IMUpacket + idx, IMUTime, 2);
    idx += 2;
    memcpy(IMUpacket + idx, IMUdata+2, 11); //DMA로 받은 필요없는 IMU데이터 2byte 버림(버리는1byte + diag_stat1byte)
    idx += 11;
    Uint16 ChSum = MakeCheckSum(IMUpacket);
    IMUpacket[REC1_SZ-1] = ChSum;

    Log_EnQueue(IMUpacket);
}
