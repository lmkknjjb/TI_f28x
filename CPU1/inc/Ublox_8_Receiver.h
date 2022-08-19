/*
 * Ublox_8_Receiver.h
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#ifndef INC_UBLOX_8_RECEIVER_H_
#define INC_UBLOX_8_RECEIVER_H_

#define UBX_COMM_SIZE       50  //Request pack size... 50-> 16bit * 50

typedef enum  {
    UBX_H1 = 0, //Header 1
    UBX_H2,     //Header 2
    UBX_CL,     //Class
    UBX_ID,     //ID
    UBX_SZ,     //Payload Size
    UBX_PL = 6, //Payloads
} UBX_Packet;

typedef struct {
    Uint32 iTow;
    Uint16 year;
    Uint16 month;
    Uint16 day;
    Uint16 hour;
    Uint16 min;
    Uint16 Sec;
    Uint16 Valid;
    Uint32 tAcc;
    int32 nano;
    Uint16 fixType;
    Uint16 flag1;
    Uint16 flag2;
    Uint16 numSV;
    int32 lon;
    int32 lat;
    float height;
    float hMSL;
    Uint32 hAcc;
    Uint32 vAcc;
    float VelN;
    float VelE;
    float VelD;
    Uint16 pDOP;
    Uint32 proTime;
    Uint16 CheckSumFLAG;
} structNEO_8N_PVT;

typedef struct {
    Uint32 proTime;
    Uint16 CheckSumFLAG;
} structNEO_8N_SAT;

extern void GET_time0(void);
void InitCommands_NEO_8N(void);
void TranLogoff_NEO_8N(void);
void GNSSSysConcurrent(void);
//void GPS_Only_NEO_8N(void);
void TranCommands_NEO_8N(void);
void BuadRateSet_NEO_8N(void);
//void Nav_5_NEO_8N(void);

void NEO_Init(void);
void PacketDeQueue_Ublox();
void TranMSG_PVT_NEO_8N(void);
void UpdateRateSet_NEO_8N(Uint16 Hz);
void Make_UBLOX_CheckSumBit(int16* pPack);
Uint16 UBLOXDataCheckSum(int16* Packet, Uint16 Size, Uint16 class, Uint16 Id);
void NAV_PVT_Decode(int16* Packet, Uint32 TIME, Uint16 size);
void TranMSG_SAT_NEO_8N(void);
void NAV_SAT_Decode(int16* Packet, Uint32 TIME, Uint16 size);

#endif /* INC_UBLOX_8_RECEIVER_H_ */
