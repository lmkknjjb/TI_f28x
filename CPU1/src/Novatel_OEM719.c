/*
 * Novatel_OEM719.c
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "inc/Global_def.h"

#ifdef NOVATEL_OEM_719

extern FlagStatus PVT_READY;
extern FlagStatus System_Start;

structOEM719_BESTXYZ    stBESTXYZ;
structOEM719_BESTXYZ*   pstBESTXYZ;

structOEM719_RANGE   stRANGE;
structOEM719_RANGE*  pstRANGE;

int16 TranCommPack[NOVATEL_COMM_SIZE] = {0x00,};   //Request pack
int16 BESTXYZpacket[REC2_SZ] = {0xFEFF, 0x0AFD, 0x0044, }; //Header:0xFFFEFFFD, ID:10, Size:... Little indian
int16 RANGEpacket[REC3_SZ] = {0xFEFF, 0x0BFD, 0x0000, }; //Header:0xFFFEFFFD, ID:11, Size: Changeable Little indian

//*******************************Timer 관련변수
extern Uint32 Small_scale_timer0;
extern Uint32 Small_scale_timer1;
extern Uint32 T0CNT0;
extern Uint32 T0CNT1;
extern Uint32 T1CNT0;
extern Uint32 T1CNT1;

void NOVATEL719_Init(void){

    pstBESTXYZ = &stBESTXYZ;
    pstRANGE = &stRANGE;
    // **************** 1. GNSS BaudrateSEt and Log-OffAll ***************** //
//    Baudrate_OEM719(); // SET Higher Baudrate
//    DELAY_US(500000);
//    TranLogoff_OEM719();
//    DELAY_US(500000);
//    VERSION_OEM719(); //Firmware Version 확인
//    DELAY_US(500000);
    // ***************** 2. GPS Command Packet Init. Start ****************** //
//    AssignALL_ALLsystem();
//    DELAY_US(500000);
//    CN0Update();
//    DELAY_US(500000);
    REQUEST_BESTXYZ_OEM719();
    DELAY_US(500000);
    REQUEST_RANGE_OEM719();
    DELAY_US(500000);
//    REQUEST_SATXYZ2_OEM719();
//    DELAY_US(100000);
//    SAVECONFIG_OEM719(); //Save current configuration to flash...현재 설정저장 - Baudrate:460800 and Unlogall(오작동을 소지가 있기 때문에 사용자제..)
//    DELAY_US(500000);
}

void SAVECONFIG_OEM719(void){

    char AbbrASCII[] = {0x4153, 0x4556, 0x4F43, 0x464E, 0x4749, 0x0A0D}; // SAVECONFIG

    ScidTxEnQueue((int16*)AbbrASCII, sizeof(AbbrASCII)*2);
}


void TranLogoff_OEM719(void){

    char AbbrASCII[5] = {0x6E75, 0x6F6C, 0x6167, 0x6C6C, 0x0D0A}; // "unlogall"
    ScidTxEnQueue((int16*)AbbrASCII, 10);
}

void CN0Update(void){

    char AbbrASCII[] = {0x4E43, 0x554F, 0x4450, 0x5441, 0x2045, 0x3032, 0x7A48, 0x0A0D}; // "CNOUPDATE 20Hz"
    ScidTxEnQueue((int16*)AbbrASCII, sizeof(AbbrASCII)*2);
}

void AssignALL_ALLsystem(void){

    char AbbrASCII[] = {0x5341, 0x4953, 0x4E47, 0x4C41, 0x204C, 0x4C41, 0x204C, 0x4341, 0x4954, 0x4556, 0x0A0D}; // "ASSIGNALL ALL ACTIVE"
    ScidTxEnQueue((int16*)AbbrASCII, sizeof(AbbrASCII)*2);
}

void VERSION_OEM719(void){

    char AbbrASCII[10] = {0x6F6C, 0x2067, 0x6576, 0x7372, 0x6F69, 0x616E, 0x6F20, 0x636E, 0x0D65, 0x000A}; // "log versiona once"
    ScidTxEnQueue((int16*)AbbrASCII, 20);
}

void REQUEST_BESTXYZ_OEM719(void){

    char AbbrASCII[] = {0x6F6C, 0x2067, 0x6F63, 0x316D, 0x6220, 0x7365, 0x7874, 0x7A79, 0x2062, 0x6E6F,
                        0x6974, 0x656D, 0x3020, 0x312E, 0x0A0D}; // "log com1 bestxyzb ontime 0.1" by Binary

//    char AbbrASCII[] = {0x6F6C, 0x2067, 0x6F63, 0x316D, 0x6220, 0x7365, 0x7874, 0x7A79, 0x6F20, 0x746E,
//                        0x6D69, 0x2065, 0x2E30, 0x3530, 0x0A0D}; // "log com1 bestxyz ontime 0.05"  by ASCII

    ScidTxEnQueue((int16*)AbbrASCII, sizeof(AbbrASCII)*2);
}

void REQUEST_RANGE_OEM719(void){

    char AbbrASCII[] = {0x6F6C, 0x2067, 0x6F63, 0x316D, 0x7220, 0x6E61, 0x6567, 0x2062, 0x6E6F, 0x6974,
                        0x656D, 0x3020, 0x312E, 0x0A0D}; // "log com1 rangeb ontime 0.1" by binary

    ScidTxEnQueue((int16*)AbbrASCII, sizeof(AbbrASCII)*2);
}

void REQUEST_SATXYZ2_OEM719(void){

    char AbbrASCII[] = {0x6F6C, 0x2067, 0x6F63, 0x316D, 0x7320, 0x7461, 0x7978, 0x327A, 0x2062, 0x6E6F,
                        0x6974, 0x656D, 0x3020, 0x312E, 0x0A0D}; // "log com1 satxyz2b ontime 0.1" by binary

    ScidTxEnQueue((int16*)AbbrASCII, sizeof(AbbrASCII)*2);
}

void Baudrate_OEM719(void){

    Uint32 CPU_FREQ  = 200E6;
    Uint32 LSPCLK_FREQ = CPU_FREQ/2;
    Uint32 SCI_FREQ = 460800; //여기서 Buadrate 설정
    Uint16 SCI_PRD = (Uint16)(LSPCLK_FREQ/(SCI_FREQ*8))-1;

    char AbbrASCII[] = {0x4553, 0x4952, 0x4C41, 0x4F43, 0x464E, 0x4749, 0x4320, 0x4D4F, 0x2031, 0x3634,
                        0x3830, 0x3030, 0x4E20, 0x3820, 0x3120, 0x4E20, 0x4F20, 0x0D4E, 0x000A}; // "SERIALCONFIG com1 460800 n 8 1 n on"


//    char AbbrASCII[] = {0x4553, 0x4952, 0x4C41, 0x4F43, 0x464E, 0x4749, 0x4320, 0x4D4F, 0x2031, 0x3131,
//                        0x3235, 0x3030, 0x4E20, 0x3820, 0x3120, 0x4E20, 0x4F20, 0x0D4E, 0x000A}; // "SERIALCONFIG com1 115200 n 8 1 n on"

    ScidTxEnQueue((int16*)AbbrASCII, sizeof(AbbrASCII)*2);
    DELAY_US(100000);

    /* SCID BAUDRATE SET 460800 */
    ScidRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScidRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScidRegs.SCICTL2.all =0x0003;

    ScidRegs.SCIHBAUD.bit.BAUD =(SCI_PRD >> 8);  // 460800 baud @LSPCLK = 100MHz
                                         //(200 MHz SYSCLK).
    ScidRegs.SCILBAUD.bit.BAUD =(SCI_PRD & 0x00FF);

    ScidRegs.SCIFFTX.all=0xC000;
    ScidRegs.SCIFFRX.all=0x0030;
    ScidRegs.SCIFFCT.all=0x0;

    ScidRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScidRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScidRegs.SCIFFRX.bit.RXFIFORESET = 1;
    /****************************/
}

void  BESTXYZ_Decode(int16* Packet, Uint32 TIME, Uint16 Headsize, Uint16 PLsize){

    Uint16 Idx = 3;
    Uint32 CRC = 0;
    Uint16 HalfHeadSz = Headsize/2;

    //Week
    stBESTXYZ.Week = *(Uint16*)(Packet + 7);
    //MilliSecond
    stBESTXYZ.GPsec = *(int32*)(Packet + 8);
    //Protime
    stBESTXYZ.proTime = TIME;
    //Psol_Stat
    stBESTXYZ.Psol_Stat = *(Uint32*)(Packet + HalfHeadSz + 0);
    //PosTYPE;
    stBESTXYZ.PosTYPE = *(Uint32*)(Packet + HalfHeadSz + 2);
    //ecefX;
    stBESTXYZ.ecefX = *(long double*)(Packet + HalfHeadSz + 4);
    //ecefY;
    stBESTXYZ.ecefY = *(long double*)(Packet + HalfHeadSz + 8);
    //ecefZ;
    stBESTXYZ.ecefZ = *(long double*)(Packet + HalfHeadSz + 12);
    //SigX;
    stBESTXYZ.SigX = *(float*)(Packet + HalfHeadSz + 16);
    //SigY;
    stBESTXYZ.SigY = *(float*)(Packet + HalfHeadSz + 18);
    //SigZ;
    stBESTXYZ.SigZ = *(float*)(Packet + HalfHeadSz + 20);
    //Vsol_Stat;
    stBESTXYZ.Vsol_Stat = *(Uint32*)(Packet + HalfHeadSz + 22);
    //VelTYPE;
    stBESTXYZ.VelTYPE = *(Uint32*)(Packet + HalfHeadSz + 24);
    //ecefvelX;
    stBESTXYZ.ecefvelX = *(long double*)(Packet + HalfHeadSz + 26);
    //ecefvelY;
    stBESTXYZ.ecefvelY = *(long double*)(Packet + HalfHeadSz + 30);
    //ecefvelZ;
    stBESTXYZ.ecefvelZ = *(long double*)(Packet + HalfHeadSz + 34);
    //SigvelX;
    stBESTXYZ.SigvelX = *(float*)(Packet + HalfHeadSz + 38);
    //SigvelY;
    stBESTXYZ.SigvelY = *(float*)(Packet + HalfHeadSz + 40);
    //SigvelZ;
    stBESTXYZ.SigvelZ = *(float*)(Packet + HalfHeadSz + 42);
    //StnID;
    stBESTXYZ.StnID = *(char*)(Packet + HalfHeadSz + 44);
    //V_latency;
    stBESTXYZ.V_latency = *(float*)(Packet + HalfHeadSz + 46);
    //diff_age;
    stBESTXYZ.diff_age = *(float*)(Packet + HalfHeadSz + 48);
    //Sol_age;
    stBESTXYZ.Sol_age = *(float*)(Packet + HalfHeadSz + 50);
    //NumSV;
    stBESTXYZ.NumSV = (Uint16)__byte(Packet, Headsize + 104);
    //NumUsedSV;
    stBESTXYZ.NumUsedSV = (Uint16)__byte(Packet, Headsize + 105);
    //NumggL1;
    stBESTXYZ.NumggL1 = (Uint16)__byte(Packet, Headsize + 106);
    //NumSolMultiSV;
    stBESTXYZ.NumSolMultiSV = (Uint16)__byte(Packet, Headsize + 107);
    //Ext_sol_stat;
    stBESTXYZ.Ext_sol_stat = (Uint16)__byte(Packet, Headsize + 109);
    //Gal_Bei_SigMask;
    stBESTXYZ.Gal_Bei_SigMask = (Uint16)__byte(Packet, Headsize + 110);
    //GPS_Glo_SigMask;
    stBESTXYZ.GPS_Glo_SigMask = (Uint16)__byte(Packet, Headsize + 111);
    //CRCcheck;
    CRC = *(Uint32*)(Packet + HalfHeadSz + 56);
    if  (CalculateBlockCRC32(Headsize + PLsize, Packet) == CRC)    stBESTXYZ.CRCcheckFlag = 1;
    else    stBESTXYZ.CRCcheckFlag = 0;


    //Week
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->Week, sizeof(pstBESTXYZ->Week));
    Idx += sizeof(pstBESTXYZ->Week);
    //MilliSecond
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->GPsec, sizeof(pstBESTXYZ->GPsec));
    Idx += sizeof(pstBESTXYZ->GPsec);
    //Protime
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->proTime, sizeof(pstBESTXYZ->proTime));
    Idx += sizeof(pstBESTXYZ->proTime);
    //Psol_Stat
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->Psol_Stat, sizeof(pstBESTXYZ->Psol_Stat));
    Idx += sizeof(pstBESTXYZ->Psol_Stat);
    //PosTYPE;
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->PosTYPE, sizeof(pstBESTXYZ->PosTYPE));
    Idx += sizeof(pstBESTXYZ->PosTYPE);
    //ecefX
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->ecefX, sizeof(pstBESTXYZ->ecefX));
    Idx += sizeof(pstBESTXYZ->ecefX);
    //ecefY
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->ecefY, sizeof(pstBESTXYZ->ecefY));
    Idx += sizeof(pstBESTXYZ->ecefY);
    //ecefZ
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->ecefZ, sizeof(pstBESTXYZ->ecefZ));
    Idx += sizeof(pstBESTXYZ->ecefZ);
    //SigX
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->SigX, sizeof(pstBESTXYZ->SigX));
    Idx += sizeof(pstBESTXYZ->SigX);
    //SigY
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->SigY, sizeof(pstBESTXYZ->SigY));
    Idx += sizeof(pstBESTXYZ->SigY);
    //SigZ
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->SigZ, sizeof(pstBESTXYZ->SigZ));
    Idx += sizeof(pstBESTXYZ->SigZ);
    //Vsol_Stat
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->Vsol_Stat, sizeof(pstBESTXYZ->Vsol_Stat));
    Idx += sizeof(pstBESTXYZ->Vsol_Stat);
    //VelTYPE
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->VelTYPE, sizeof(pstBESTXYZ->VelTYPE));
    Idx += sizeof(pstBESTXYZ->VelTYPE);
    //ecefvelX
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->ecefvelX, sizeof(pstBESTXYZ->ecefvelX));
    Idx += sizeof(pstBESTXYZ->ecefvelX);
    //ecefvelY
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->ecefvelY, sizeof(pstBESTXYZ->ecefvelY));
    Idx += sizeof(pstBESTXYZ->ecefvelY);
    //ecefvelZ
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->ecefvelZ, sizeof(pstBESTXYZ->ecefvelZ));
    Idx += sizeof(pstBESTXYZ->ecefvelZ);
    //SigvelX
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->SigvelX, sizeof(pstBESTXYZ->SigvelX));
    Idx += sizeof(pstBESTXYZ->SigvelX);
    //SigvelY
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->SigvelY, sizeof(pstBESTXYZ->SigvelY));
    Idx += sizeof(pstBESTXYZ->SigvelY);
    //SigvelZ
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->SigvelZ, sizeof(pstBESTXYZ->SigvelZ));
    Idx += sizeof(pstBESTXYZ->SigvelZ);
    //StnID
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->StnID, sizeof(pstBESTXYZ->StnID));
    Idx += sizeof(pstBESTXYZ->StnID);
    //V_latency
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->V_latency, sizeof(pstBESTXYZ->V_latency));
    Idx += sizeof(pstBESTXYZ->V_latency);
    //diff_age
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->diff_age, sizeof(pstBESTXYZ->diff_age));
    Idx += sizeof(pstBESTXYZ->diff_age);
    //Sol_age
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->Sol_age, sizeof(pstBESTXYZ->Sol_age));
    Idx += sizeof(pstBESTXYZ->Sol_age);
    //NumSV
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->NumSV, sizeof(pstBESTXYZ->NumSV));
    Idx += sizeof(pstBESTXYZ->NumSV);
    //NumUsedSV
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->NumUsedSV, sizeof(pstBESTXYZ->NumUsedSV));
    Idx += sizeof(pstBESTXYZ->NumUsedSV);
    //NumggL1
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->NumggL1, sizeof(pstBESTXYZ->NumggL1));
    Idx += sizeof(pstBESTXYZ->NumggL1);
    //NumSolMultiSV
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->NumSolMultiSV, sizeof(pstBESTXYZ->NumSolMultiSV));
    Idx += sizeof(pstBESTXYZ->NumSolMultiSV);
    //Ext_sol_stat
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->Ext_sol_stat, sizeof(pstBESTXYZ->Ext_sol_stat));
    Idx += sizeof(pstBESTXYZ->Ext_sol_stat);
    //Gal_Bei_SigMask
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->Gal_Bei_SigMask, sizeof(pstBESTXYZ->Gal_Bei_SigMask));
    Idx += sizeof(pstBESTXYZ->Gal_Bei_SigMask);
    //GPS_Glo_SigMask
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->GPS_Glo_SigMask, sizeof(pstBESTXYZ->GPS_Glo_SigMask));
    Idx += sizeof(pstBESTXYZ->GPS_Glo_SigMask);
    //CRCcheckFlag
    memcpy(BESTXYZpacket + Idx, &pstBESTXYZ->CRCcheckFlag, sizeof(pstBESTXYZ->CRCcheckFlag));

    //SDCARD QUEUE SAVE
    //아래 sd카드 저장은 구조체를 받아서 바로 처리하면 더욱 효율적으로 코드를 구현할 수 있지만
    //구조체 선언시 각 변수의 크기에 따라 정렬이 변태같기 때문에 위에서 PVTpack에 바로 저장한다(코드의 일반화를 위해...)
    if(System_Start){
        Log_EnQueue2(BESTXYZpacket);
    }
    //This FLAG Will be RESET WHEN Used at CPU2
    DEF_SETFLAG(PVT_READY);
}

void RANGE_Decode(int16* Packet, Uint32 TIME, Uint16 Headsize, Uint16 PLsize){

    Uint16 Idx = 3;
    Uint16 HalfHeadSz = Headsize/2;
    Uint16 RANGE_Changeable_Size = PLsize/2; // SAT는 사이즈가 항상 짝수이기 때문에 나누어줘도 문제없다.

    //limit the satellite Number to 40
    if(RANGE_Changeable_Size > 882){
        RANGE_Changeable_Size = 882;
    }

    //RANGE original Size + WEEK(1) + GPsec(2) + MCUTIME(2) + CRC(2) + Header,ID,SZ(3)
    RANGEpacket[2] = RANGE_Changeable_Size + 10;

    //Week
    stRANGE.Week = *(Uint16*)(Packet + 7);
    //MilliSecond
    stRANGE.GPsec = *(int32*)(Packet + 8);
    //Protime
    stRANGE.proTime = TIME;
    //CRC..데이터가 너무 클 수 있기 때문에 여기서 CRC체크는 하지 않는다//위성갯수 overflow시 CRC는 받지 않는다..따로 코드작성해야..
    stRANGE.CRC = *(Uint32*)(Packet + HalfHeadSz + (PLsize/2));

    //Week
    memcpy(RANGEpacket + Idx, &pstRANGE->Week, sizeof(pstRANGE->Week));
    Idx += sizeof(pstRANGE->Week);
    //MilliSecond
    memcpy(RANGEpacket + Idx, &pstRANGE->GPsec, sizeof(pstRANGE->GPsec));
    Idx += sizeof(pstRANGE->GPsec);
    //Protime
    memcpy(RANGEpacket + Idx, &pstRANGE->proTime, sizeof(pstRANGE->proTime));
    Idx += sizeof(pstRANGE->proTime);
    //CRC
    memcpy(RANGEpacket + Idx, &pstRANGE->CRC, sizeof(pstRANGE->CRC));
    Idx += sizeof(pstRANGE->CRC);
    //ALL DATA
    memcpy(RANGEpacket + Idx, Packet + HalfHeadSz, RANGE_Changeable_Size);

    if(System_Start){
        Log_EnQueue3(RANGEpacket);
    }
}


Uint32 CRC32Value(Uint16 i) {
    int16 j;
    Uint32 ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- ) {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

Uint32 CalculateBlockCRC32(Uint16 ulCount, int16 *ucBuffer ) {
    Uint32 ulTemp1;
    Uint32 ulTemp2;
    Uint32 ulCRC = 0;
    Uint16 j = 0;

    while ( ulCount-- != 0 ) {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ( (int16)ulCRC ^ __byte(ucBuffer,j++) ) & 0xFF );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}

#endif
