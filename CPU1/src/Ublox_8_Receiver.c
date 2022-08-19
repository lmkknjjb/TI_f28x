/*
 * Ublox_8_Receiver.c
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "inc/Global_def.h"

#ifdef UBLOX_NEO_8

extern FlagStatus PVT_READY;
extern FlagStatus System_Start;

structNEO_8N_PVT    stPVT;
structNEO_8N_PVT*   pstPVT;

structNEO_8N_SAT    stSAT;
structNEO_8N_SAT*   pstSAT;

int16 TranCommPack[UBX_COMM_SIZE] = {0x00,};   //Request pack
int16 PVTpacket[REC2_SZ] = {0xFEFF, 0x06FD, 0x002A, }; //Header:0xFFFEFFFD, ID:6, Size:42(0x2A).. Little indian
int16 SATpacket[REC3_SZ] = {0xFEFF, 0x07FD, 0x0000, }; //Header:0xFFFEFFFD, ID:7, Size:Change with SVnumber.. Little indian

//*******************************Timer 관련변수
extern Uint32 Small_scale_timer0;
extern Uint32 Small_scale_timer1;
extern Uint32 T0CNT0;
extern Uint32 T0CNT1;
extern Uint32 T1CNT0;
extern Uint32 T1CNT1;


void NEO_Init(void){

    pstPVT = &stPVT;
    pstSAT = &stSAT;
    // **************** 1. GPS Log-Off Start ***************** //
    TranLogoff_NEO_8N();
//    GNSSSysConcurrent();
    // ***************** 2. GPS Command Packet Init. Start ****************** //
    TranMSG_PVT_NEO_8N();
    DELAY_US(100000);
    TranMSG_SAT_NEO_8N();
    // ***************** 3. GPS Command transmission Start ****************** //
    DELAY_US(100000);
    InitCommands_NEO_8N();

}


void TranLogoff_NEO_8N(void){

    Uint16 HCI[2] = {0x62b5, 0x0106}; //Header(2), class(1), ID(1)
    Uint16 size = 3;
    Uint16 Payload = UBX_PL;
    Uint16 i = 0;

    memset(TranCommPack, 0x00, sizeof(TranCommPack));
    // CFG-MSG : Set Message Rate
    memcpy(TranCommPack, &HCI, 2);
    memcpy(TranCommPack + 2, &size, 1);
    // msgClass
    __byte(TranCommPack, Payload + 0) = 0xF0;
    // rate
    __byte(TranCommPack, Payload + 2) = 0x0;

    for(i=0;i<6;i++){
        __byte(TranCommPack, Payload + 1) = i; // msgID
        Make_UBLOX_CheckSumBit(TranCommPack);
        TranCommands_NEO_8N();
        DELAY_US(100000);
    }
}

void GNSSSysConcurrent(void){

    Uint16 HCI[2] = {0x62b5, 0x3E06}; //Header(2), class(1), ID(1)
    Uint16 size = 44;
    Uint16 Payload = UBX_PL;
    Uint32 GPSFlag = 65537;
    Uint32 GloFlag = 65537;
    Uint32 GalFlag = 65537;
    Uint32 QZSSFlag = 65537;
    Uint32 SBASFlag = 65537;

    memset(TranCommPack, 0x00, sizeof(TranCommPack));
    memcpy(TranCommPack, &HCI, 2);
    memcpy(TranCommPack + 2, &size, 1);
    // msgVer
    __byte(TranCommPack, Payload + 0) = 0;
    // numTrkchUse(readonly)
    __byte(TranCommPack, Payload + 1) = 0x0;
    // numTrkchUse..maximum = FF
    __byte(TranCommPack, Payload + 2) = 0xFF;
    // numof Setting GNSS SYSTEM ..(GPS, Glonass, Galileo, SBAS, QZSS) five GNSS system
    __byte(TranCommPack, Payload + 3) = 5;
    //************** GPS ****************//
    __byte(TranCommPack, 0 + Payload + 4) = 0;
    // ResTrkCh
    __byte(TranCommPack, 0 + Payload + 5) = 8;
    // MaxTrkCh
    __byte(TranCommPack, 0 + Payload + 6) = 16;
    // Reserved1
    __byte(TranCommPack, 0 + Payload + 7) = 0;
    // Flags
    memcpy(TranCommPack + 0 + (Payload/2) + 4, &GPSFlag, 2);

    //************** SBAS ****************//
    __byte(TranCommPack, 8 + Payload + 4) = 1;
    // ResTrkCh
    __byte(TranCommPack, 8 + Payload + 5) = 1;
    // MaxTrkCh
    __byte(TranCommPack, 8 + Payload + 6) = 3;
    // Reserved1
    __byte(TranCommPack, 8 + Payload + 7) = 0;
    // Flags
    memcpy(TranCommPack + 4 + (Payload/2) + 4, &SBASFlag, 2);

    //************** Galileo ****************//
    __byte(TranCommPack, 16 + Payload + 4) = 2;
    // ResTrkCh
    __byte(TranCommPack, 16 + Payload + 5) = 4;
    // MaxTrkCh
    __byte(TranCommPack, 16 + Payload + 6) = 10;
    // Reserved1
    __byte(TranCommPack, 16 + Payload + 7) = 0;
    // Flags
    memcpy(TranCommPack + 8 + (Payload/2) + 4, &GalFlag, 2);

    //************** QZSS ****************//
    __byte(TranCommPack, 24 + Payload + 4) = 5;
    // ResTrkCh
    __byte(TranCommPack, 24 + Payload + 5) = 0;
    // MaxTrkCh
    __byte(TranCommPack, 24 + Payload + 6) = 3;
    // Reserved1
    __byte(TranCommPack, 24 + Payload + 7) = 0;
    // Flags
    memcpy(TranCommPack + 12 + (Payload/2) + 4, &QZSSFlag, 2);

    //************** Glonass ****************//
    __byte(TranCommPack, 32 + Payload + 4) = 6;
    // ResTrkCh
    __byte(TranCommPack, 32 + Payload + 5) = 8;
    // MaxTrkCh
    __byte(TranCommPack, 32 + Payload + 6) = 14;
    // Reserved1
    __byte(TranCommPack, 32 + Payload + 7) = 0;
    // Flags
    memcpy(TranCommPack + 16 + (Payload/2) + 4, &GloFlag, 2);

    Make_UBLOX_CheckSumBit(TranCommPack);

    TranCommands_NEO_8N();
}

void BuadRateSet_NEO_8N(void){

    Uint16 HCI[2] = {0x62b5, 0x0006}; //Header(2), class(1), ID(1)
    Uint16 size = 20;
    Uint16 Payload = UBX_PL;

    Uint16 PortID = 0x1; //8bits
    Uint16 txReady = 0x0; //16bits
    Uint32 Mode = 0x08C0; //32bits
    Uint32 Buadrate = 921600; //32bits
    Uint16 inProtoMask = 0x3; //16bits
    Uint16 outProtoMask = 0x3; //16bits
    Uint16 flags = 0x0; //16bits

    memset(TranCommPack, 0x00, sizeof(TranCommPack));
    // ===== Buad-Rate : 115200 ===== //
    memcpy(TranCommPack, &HCI, 2);
    memcpy(TranCommPack + 2, &size, 1);
    //Port ID
    __byte(TranCommPack, Payload + 0) = PortID;
    //txReady
    memcpy(TranCommPack + (Payload/2 + 1), &txReady, 1);
    //Mode
    memcpy(TranCommPack + (Payload/2 + 2), &Mode, 2);
    //BuadRate
    memcpy(TranCommPack + (Payload/2 + 4), &Buadrate, 2);
    //inProtoMask
    memcpy(TranCommPack + (Payload/2 + 6), &inProtoMask,   1);
    //outProtoMask
    memcpy(TranCommPack + (Payload/2 + 7), &outProtoMask,  1);
    //flags
    memcpy(TranCommPack + (Payload/2 + 8), &flags,         1);
    //Checksum
    Make_UBLOX_CheckSumBit(TranCommPack);

    TranCommands_NEO_8N();
}

void UpdateRateSet_NEO_8N(Uint16 Hz){

    Uint16 HCI[2] = {0x62b5, 0x0806}; //Header(2), class(1), ID(1)
    Uint16 size = 6;
    Uint16 Payload = UBX_PL;

    Uint16 Period = (Uint16)(1000/Hz);
    Uint16 navRate = 0x1; //
    Uint16 timeRef = 0x0; //(UTC TIME)

    memset(TranCommPack, 0x00, sizeof(TranCommPack));
    // ===== Navigation Solution Rate 5Hz ===== //
    memcpy(TranCommPack, &HCI, 2);
    memcpy(TranCommPack + 2, &size, 1);
    //measRate
    memcpy(TranCommPack + Payload/2 + 0,   &Period,  1);
    //navRate
    memcpy(TranCommPack + Payload/2 + 1,   &navRate, 1);
    //timeRef
    memcpy(TranCommPack + Payload/2 + 2,   &timeRef, 1);
    //Checksum
    Make_UBLOX_CheckSumBit(TranCommPack);

    TranCommands_NEO_8N();
}

void InitCommands_NEO_8N(void){

    Uint32 CPU_FREQ  = 200E6;
    Uint32 LSPCLK_FREQ = CPU_FREQ/2;
    Uint32 SCI_FREQ = 921600; //여기서 Buadrate 설정
    Uint16 SCI_PRD = (Uint16)(LSPCLK_FREQ/(SCI_FREQ*8))-1;

    Uint16 Set_Hz = 20;

    BuadRateSet_NEO_8N();
    DELAY_US(100000);

    /* SCID BAUDRATE SET 115200 */
    ScidRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScidRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScidRegs.SCICTL2.all =0x0003;

    ScidRegs.SCIHBAUD.bit.BAUD =(SCI_PRD >> 8);  // 115200 baud @LSPCLK = 100MHz
                                         //(200 MHz SYSCLK).
    ScidRegs.SCILBAUD.bit.BAUD =(SCI_PRD & 0x00FF);

    ScidRegs.SCIFFTX.all=0xC000;
    ScidRegs.SCIFFRX.all=0x0030;
    ScidRegs.SCIFFCT.all=0x0;

    ScidRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScidRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScidRegs.SCIFFRX.bit.RXFIFORESET = 1;
    /****************************/

    //Update Frequency Set - 5 Hz
    UpdateRateSet_NEO_8N(Set_Hz);
    DELAY_US(100000);
}

void TranMSG_PVT_NEO_8N(void){

    Uint16 HCI[2] = {0x62b5, 0x0106}; //Header(2), class(1), ID(1)
    Uint16 size = 3;
    Uint16 Payload = UBX_PL;

    memset(TranCommPack, 0x00, sizeof(TranCommPack));
    // CFG-MSG : Set Message Rate
    memcpy(TranCommPack, &HCI, 2);
    memcpy(TranCommPack + 2, &size, 1);
    // PVT Class ID request
    __byte(TranCommPack, Payload + 0) = 0x1;
    //
    __byte(TranCommPack, Payload + 1) = 0x7;
    //
    __byte(TranCommPack, Payload + 2) = 0x1;
    //Checksum
    Make_UBLOX_CheckSumBit(TranCommPack);
    TranCommands_NEO_8N();
}

void TranMSG_SAT_NEO_8N(void){

    Uint16 HCI[2] = {0x62b5, 0x0106}; //Header(2), class(1), ID(1)
    Uint16 size = 3;
    Uint16 Payload = UBX_PL;

    memset(TranCommPack, 0x00, sizeof(TranCommPack));
    // CFG-MSG : Set Message Rate
    memcpy(TranCommPack, &HCI, 2);
    memcpy(TranCommPack + 2, &size, 1);
    // SAT Class ID request
    __byte(TranCommPack, Payload + 0) = 0x1;
    //
    __byte(TranCommPack, Payload + 1) = 0x35;
    //
    __byte(TranCommPack, Payload + 2) = 0x1;
    //Checksum
    Make_UBLOX_CheckSumBit(TranCommPack);
    TranCommands_NEO_8N();
}


void  NAV_PVT_Decode(int16* Packet, Uint32 TIME, Uint16 size){

    Uint16 Idx = 3;
    //MilliSecond
    stPVT.iTow = *(Uint32*)(Packet + 0);
    memcpy(PVTpacket + Idx, &pstPVT->iTow, sizeof(pstPVT->iTow));
    Idx += sizeof(pstPVT->iTow);

    //Year
    stPVT.year = *(Uint16*)(Packet + 2);
    memcpy(PVTpacket + Idx, &pstPVT->year, sizeof(pstPVT->year));
    Idx += sizeof(pstPVT->year);

    //Month
    stPVT.month = (Uint16)__byte(Packet, 6);
    memcpy(PVTpacket + Idx, &pstPVT->month, sizeof(pstPVT->month));
    Idx += sizeof(pstPVT->month);

    //day
    stPVT.day = (Uint16)__byte(Packet, 7);
    memcpy(PVTpacket + Idx, &pstPVT->day, sizeof(pstPVT->day));
    Idx += sizeof(pstPVT->day);

    //hour
    stPVT.hour = (Uint16)__byte(Packet, 8);
    memcpy(PVTpacket + Idx, &pstPVT->hour, sizeof(pstPVT->hour));
    Idx += sizeof(pstPVT->hour);

    //min
    stPVT.min = (Uint16)__byte(Packet, 9);
    memcpy(PVTpacket + Idx, &pstPVT->min, sizeof(pstPVT->min));
    Idx += sizeof(pstPVT->min);

    //sec
    stPVT.Sec = (Uint16)__byte(Packet, 10);
    memcpy(PVTpacket + Idx, &pstPVT->Sec, sizeof(pstPVT->Sec));
    Idx += sizeof(pstPVT->Sec);

    //Valid
    stPVT.Valid = (Uint16)__byte(Packet, 11);
    memcpy(PVTpacket + Idx, &pstPVT->Valid, sizeof(pstPVT->Valid));
    Idx += sizeof(pstPVT->Valid);

    //tAcc
    stPVT.tAcc = *(Uint32*)(Packet + 6); // NanoSec
    memcpy(PVTpacket + Idx, &pstPVT->tAcc, sizeof(pstPVT->tAcc));
    Idx += sizeof(pstPVT->tAcc);

    //nano
    stPVT.nano = *(int32*)(Packet + 8); // NanoSec
    memcpy(PVTpacket + Idx, &pstPVT->nano, sizeof(pstPVT->nano));
    Idx += sizeof(pstPVT->nano);

    //fixType
    stPVT.fixType = (Uint16)__byte(Packet, 20);
    memcpy(PVTpacket + Idx, &pstPVT->fixType, sizeof(pstPVT->fixType));
    Idx += sizeof(pstPVT->fixType);

    //flag1
    stPVT.flag1 = (Uint16)__byte(Packet, 21);
    memcpy(PVTpacket + Idx, &pstPVT->flag1, sizeof(pstPVT->flag1));
    Idx += sizeof(pstPVT->flag1);

    //flag2
    stPVT.flag2 = (Uint16)__byte(Packet, 22);
    memcpy(PVTpacket + Idx, &pstPVT->flag2, sizeof(pstPVT->flag2));
    Idx += sizeof(pstPVT->flag2);

    //numSV
    stPVT.numSV = (Uint16)__byte(Packet, 23);
    memcpy(PVTpacket + Idx, &pstPVT->numSV, sizeof(pstPVT->numSV));
    Idx += sizeof(pstPVT->numSV);

    //lon
    stPVT.lon = *(int32*)(Packet + 12);
    memcpy(PVTpacket + Idx, &pstPVT->lon, sizeof(pstPVT->lon));
    Idx += sizeof(pstPVT->lon);

    //lat
    stPVT.lat = *(int32*)(Packet + 14);
    memcpy(PVTpacket + Idx, &pstPVT->lat, sizeof(pstPVT->lat));
    Idx += sizeof(pstPVT->lat);

    //height
    stPVT.height = 1e-3*(float)*(int32*)(Packet + 16);
    memcpy(PVTpacket + Idx, &pstPVT->height, sizeof(pstPVT->height));
    Idx += sizeof(pstPVT->height);

    //hMSL
    stPVT.hMSL = 1e-3*(float)*(int32*)(Packet + 18);
    memcpy(PVTpacket + Idx, &pstPVT->hMSL, sizeof(pstPVT->hMSL));
    Idx += sizeof(pstPVT->hMSL);

    //hAcc
    stPVT.hAcc = *(Uint32*)(Packet + 20); //mm
    memcpy(PVTpacket + Idx, &pstPVT->hAcc, sizeof(pstPVT->hAcc));
    Idx += sizeof(pstPVT->hAcc);

    //vAcc
    stPVT.vAcc = *(Uint32*)(Packet + 22); ////mm
    memcpy(PVTpacket + Idx, &pstPVT->vAcc, sizeof(pstPVT->vAcc));
    Idx += sizeof(pstPVT->vAcc);

    //velN
    stPVT.VelN = 1e-3*(float)*(int32*)(Packet + 24);
    memcpy(PVTpacket + Idx, &pstPVT->VelN, sizeof(pstPVT->VelN));
    Idx += sizeof(pstPVT->VelN);

    //velE
    stPVT.VelE = 1e-3*(float)*(int32*)(Packet + 26);
    memcpy(PVTpacket + Idx, &pstPVT->VelE, sizeof(pstPVT->VelE));
    Idx += sizeof(pstPVT->VelE);

    //velD
    stPVT.VelD = 1e-3*(float)*(int32*)(Packet + 28);
    memcpy(PVTpacket + Idx, &pstPVT->VelD, sizeof(pstPVT->VelD));
    Idx += sizeof(pstPVT->VelD);

    //pDOP
    stPVT.pDOP = *(Uint16*)(Packet + 38); // Scale Factor Multiplication by 0.01 needed
    memcpy(PVTpacket + Idx, &pstPVT->pDOP, sizeof(pstPVT->pDOP));
    Idx += sizeof(pstPVT->pDOP);

    //CheckSumFLAG
    stPVT.CheckSumFLAG = UBLOXDataCheckSum(Packet, size, 0x01, 0x07);
    memcpy(PVTpacket + Idx, &pstPVT->CheckSumFLAG, sizeof(pstPVT->CheckSumFLAG));
    Idx += sizeof(pstPVT->CheckSumFLAG);

    //MCU TIME CHECK
    stPVT.proTime = TIME;
    memcpy(PVTpacket + Idx, &pstPVT->proTime, sizeof(pstPVT->proTime));

    /*******************Not development***************
    //gSpeed
    stPVT.gSpeed
    //sAcc
    stPVT.sAcc
    //headMot
    stPVT.HeadMot
    //HeadAcc
    stPVT.HeadAcc
    //HeadVeh
    stPVT.HeadVeh
    //magDec
    stPVT.magDec
    //magAcc
    stPVT.magAcc
    *******************Not development***************/
    //SDCARD QUEUE SAVE
    //아래 sd카드 저장은 구조체를 받아서 바로 처리하면 더욱 효율적으로 코드를 구현할 수 있지만
    //구조체 선언시 각 변수의 크기에 따라 정렬이 변태같기 때문에 위에서 PVTpack에 바로 저장한다(코드의 일반화를 위해...)
    if(System_Start){
            Log_EnQueue2(PVTpacket);
    }
    //This FLAG Will be RESET WHEN Used at CPU2
    DEF_SETFLAG(PVT_READY);
}

void  NAV_SAT_Decode(int16* Packet, Uint32 TIME, Uint16 size){

    // THis Function Not decode, Only Data SAVE!!
    // IF NEEDED Write Decode code

    Uint16 Idx = 3;
    Uint16 SAT_Changeable_Size = size/2; // SAT는 사이즈가 항상 짝수이기 때문에 나누어줘도 문제없다.

    //limit the satellite Number to 70
    if(SAT_Changeable_Size > 424){
        SAT_Changeable_Size = 424;
    }

    //SAT original Size + MCUTIME and ChecksumFlag size + Header, Id, SZ
    SATpacket[2] = SAT_Changeable_Size + 6;

    //CheckSumFLAG
    stSAT.CheckSumFLAG = UBLOXDataCheckSum(Packet, size, 0x01, 0x35);
    memcpy(SATpacket + Idx, &pstSAT->CheckSumFLAG, sizeof(pstSAT->CheckSumFLAG));
    Idx += sizeof(pstSAT->CheckSumFLAG);

    //MCU TIME CHECK
    stSAT.proTime = TIME;
    memcpy(SATpacket + Idx, &pstSAT->proTime, sizeof(pstSAT->proTime));
    Idx += sizeof(pstSAT->proTime);

    //ALL DATA
    memcpy(SATpacket + Idx, Packet, SAT_Changeable_Size);

    //SDCARD QUEUE SAVE
    if(System_Start){
    Log_EnQueue3(SATpacket);
    }

}


void TranCommands_NEO_8N(void){
    //size of Request pack
    Uint16 PackSize = TranCommPack[2];
    //Enqueue of pack-> Payload size and plus 8*8bit(Head(2),Class(1),Id(1),Size(2),Checksum(2))
    ScidTxEnQueue(TranCommPack, PackSize + 8);
}

void Make_UBLOX_CheckSumBit(int16* pPack){
    Uint16 i = 0;
    Uint16 CK_A = 0x0, CK_B = 0x0;
    Uint16 Size = pPack[2] + 6;

    for(i = 2; i < Size; i++){
        CK_A += __byte(pPack,i);
        CK_B += CK_A;
    }

    __byte(pPack, Size)     = CK_A;
    __byte(pPack, Size+1)   = CK_B;
}


Uint16 UBLOXDataCheckSum(int16* Packet, Uint16 Size, Uint16 class, Uint16 Id){

    Uint16 i = 0;
    Uint16 Flag = 0;
    Uint16 CK_A = 0x0, CK_B = 0x0;
    Uint16 CheckSumDataA = __byte(Packet, Size);    //CheckSum DataA
    Uint16 CheckSumDataB = __byte(Packet, Size+1);    //CheckSum DataB

    CK_A += class;
    CK_B += CK_A;
    CK_A += Id;
    CK_B += CK_A;
    CK_A += (Size & 0x00FF);
    CK_B += CK_A;
    CK_A += (Size & 0xFF00) >> 8;
    CK_B += CK_A;

    for(i = 0; i < Size; i++){
        CK_A += __byte(Packet,i);
        CK_B += CK_A;
    }
    CK_A = (CK_A & 0x00FF);
    CK_B = (CK_B & 0x00FF);
    if( (CheckSumDataA == CK_A) && (CheckSumDataB == CK_B) )     Flag = 1;

    return Flag;
}

#endif
