/*
 * Novatel_OEM719.h
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#ifndef INC_NOVATEL_OEM719_H_
#define INC_NOVATEL_OEM719_H_

#define CRC32_POLYNOMIAL 0xEDB88320L
#define NOVATEL_COMM_SIZE       50  //Request pack size... 50-> 16bit * 50

typedef enum {
    Nov_H1 = 0, //Header 1
    Nov_H2,     //Header 2
    Nov_H3,     //Header 3
    Nov_Hlen,   //HeaderLength
    Nov_ID,     //ID
    Nov_SZ = 8, //Payloads
    Nov_PL = 10,
} Nov_Packet;

typedef struct {
    Uint16 Week;
    int32 GPsec;
    Uint32 Psol_Stat;
    Uint32 PosTYPE;
    long double ecefX;
    long double ecefY;
    long double ecefZ;
    float SigX;
    float SigY;
    float SigZ;
    Uint32 Vsol_Stat;
    Uint32 VelTYPE;
    long double ecefvelX;
    long double ecefvelY;
    long double ecefvelZ;
    float SigvelX;
    float SigvelY;
    float SigvelZ;
    Uint32 StnID;
    float V_latency;
    float diff_age;
    float Sol_age;
    Uint16 NumSV;
    Uint16 NumUsedSV;
    Uint16 NumggL1;
    Uint16 NumSolMultiSV;
    Uint16 Ext_sol_stat;
    Uint16 Gal_Bei_SigMask;
    Uint16 GPS_Glo_SigMask;
    Uint32 proTime;
    Uint16 CRCcheckFlag;
} structOEM719_BESTXYZ;

typedef struct {
    Uint16 Week;
    int32 GPsec;
    Uint32 proTime;
    Uint32 CRC;
} structOEM719_RANGE;

extern void GET_time0(void);
void NOVATEL719_Init(void);
void SAVECONFIG_OEM719(void);
void TranLogoff_OEM719(void);
void VERSION_OEM719(void);
void CN0Update(void);
void AssignALL_ALLsystem(void);
void Baudrate_OEM719(void);
void REQUEST_BESTXYZ_OEM719(void);
void REQUEST_RANGE_OEM719(void);
void REQUEST_SATXYZ2_OEM719(void);
void BESTXYZ_Decode(int16* Packet, Uint32 TIME, Uint16 Headsize, Uint16 PLsize);
void RANGE_Decode(int16* Packet, Uint32 TIME, Uint16 Headsize, Uint16 PLsize);
Uint32 CRC32Value(Uint16 i);
Uint32 CalculateBlockCRC32(Uint16 ulCount, int16 *ucBuffer );

#endif /* INC_NOVATEL_OEM719_H_ */
