/*
 * SDCARD_driver.h
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#ifndef INC_SDCARD_DRIVER_H_
#define INC_SDCARD_DRIVER_H_


void SD_DataStorage(int16* pPack, Uint16 PackSize);
void SD_Sync(void);
void SD_Initialize(char* pch);
void CreateNewFile(char * pch);

#endif /* INC_SDCARD_DRIVER_H_ */
