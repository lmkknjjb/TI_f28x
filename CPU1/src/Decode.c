/*
 * Decode.c
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "inc/Global_def.h"

Uint16 MakeCheckSum(int* UpdatePacket){
    int i = 0;
    Uint16 Result = 0;

    for(i=6; i < (REC1_SZ-1); i++){ //Only data(exclude header, id, size)
        Result ^= UpdatePacket[i];
    }
    return Result;
}
