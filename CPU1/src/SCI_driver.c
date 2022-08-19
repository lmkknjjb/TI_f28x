/*
 * SCI_driver.c
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "inc/Global_def.h"

structSCIQueue      SCIATXqueue;
structSCIQueue*     pSCIATXqueue;

structSCIQueue      SCIARXqueue;
structSCIQueue*     pSCIARXqueue;

structSCIQueue      SCIBTXqueue;
structSCIQueue*     pSCIBTXqueue;

structSCIQueue      SCIBRXqueue;
structSCIQueue*     pSCIBRXqueue;

structSCIQueue      SCICTXqueue;
structSCIQueue*     pSCICTXqueue;

structSCIQueue      SCICRXqueue;
structSCIQueue*     pSCICRXqueue;

structSCIQueue      SCIDTXqueue;
structSCIQueue*     pSCIDTXqueue;

structSCIQueue      SCIDRXqueue;
structSCIQueue*     pSCIDRXqueue;

Uint16 MAX_FIFO_size = 16;
//*******************************Timer 관련변수
extern Uint32 Small_scale_timer0;
extern Uint32 Small_scale_timer1;
extern Uint32 T0CNT0;
extern Uint32 T0CNT1;
extern Uint32 T1CNT0;
extern Uint32 T1CNT1;
Uint32 MCU_TIME = 0;

void InitScia(void){

    float CPU_FREQ  = 200E6;
    float LSPCLK_FREQ = CPU_FREQ/2;
    float SCI_FREQ = 9600; //여기서 Buadrate 설정
    Uint16 SCI_PRD = (Uint16)(LSPCLK_FREQ/(SCI_FREQ*8))-1;

    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all =0x0003;

    SciaRegs.SCIHBAUD.bit.BAUD =(SCI_PRD >> 8);  // 9600 baud @LSPCLK = 100MHz
                                         //(200 MHz SYSCLK).
    SciaRegs.SCILBAUD.bit.BAUD =(SCI_PRD & 0x00FF);

    SciaRegs.SCIFFTX.all=0xC000;
    SciaRegs.SCIFFRX.all=0x0030;
    ScidRegs.SCIFFCT.all=0x0;

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

void InitSciaQueue(void){

    Uint16 TXQueSZ = {SCIA_TXQUE/2};
    Uint16 RXQueSZ = {SCIA_RXQUE/2};
    Uint16 RXDecodeQueSZ = {SCIA_DecodeQUE};

    pSCIATXqueue = &SCIATXqueue;
    memset(&SCIATXqueue, 0x0, sizeof(SCIATXqueue));
    SCIATXqueue.SCI_Q_SIZE = TXQueSZ;
    pSCIATXqueue->ref_8bit_Qsize = SCIA_TXQUE;

    pSCIATXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * TXQueSZ);

    pSCIARXqueue = &SCIARXqueue;
    memset(&SCIARXqueue, 0x0, sizeof(SCIARXqueue));
    SCIARXqueue.SCI_Q_SIZE = RXQueSZ;
    pSCIARXqueue->ref_8bit_Qsize = SCIA_RXQUE;

    pSCIARXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * RXQueSZ);

    pSCIARXqueue->Decode_Buffer = (int16*)malloc(sizeof(int16) * RXDecodeQueSZ);
}

void SciaTxQueueFree(void){

    free(pSCIATXqueue->SCI_pstQarr);
}


void InitScib(void){

    float CPU_FREQ  = 200E6;
    float LSPCLK_FREQ = CPU_FREQ/2;
    float SCI_FREQ = 9600; //여기서 Buadrate 설정
    Uint16 SCI_PRD = (Uint16)(LSPCLK_FREQ/(SCI_FREQ*8))-1;

    ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all =0x0003;

    ScibRegs.SCIHBAUD.bit.BAUD =(SCI_PRD >> 8);  // 9600 baud @LSPCLK = 100MHz
                                         //(200 MHz SYSCLK).
    ScibRegs.SCILBAUD.bit.BAUD =(SCI_PRD & 0x00FF);

    ScibRegs.SCIFFTX.all=0xC000;
    ScibRegs.SCIFFRX.all=0x0030;
    ScibRegs.SCIFFCT.all=0x0;

    ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

void InitScibQueue(void){

    Uint16 TXQueSZ = {SCIB_TXQUE/2};
    Uint16 RXQueSZ = {SCIB_RXQUE/2};
    Uint16 RXDecodeQueSZ = {SCIB_DecodeQUE};

    pSCIBTXqueue = &SCIBTXqueue;
    memset(&SCIBTXqueue, 0x0, sizeof(SCIBTXqueue));
    SCIBTXqueue.SCI_Q_SIZE = TXQueSZ;
    pSCIBTXqueue->ref_8bit_Qsize = SCIB_TXQUE;

    pSCIBTXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * TXQueSZ);

    pSCIBRXqueue = &SCIBRXqueue;
    memset(&SCIBRXqueue, 0x0, sizeof(SCIBRXqueue));
    SCIBRXqueue.SCI_Q_SIZE = RXQueSZ;
    pSCIBRXqueue->ref_8bit_Qsize = SCIB_RXQUE;

    pSCIBRXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * RXQueSZ);

    pSCIBRXqueue->Decode_Buffer = (int16*)malloc(sizeof(int16) * RXDecodeQueSZ);
}

void ScibTxQueueFree(void){

    free(pSCIBTXqueue->SCI_pstQarr);
}

void ScibTxEnQueue(int16 *buf, Uint16 size){
    /*여기서 입력값으로 받는 size는 8bit 기준이다*/
    /*버퍼의 크기(size)는 항상 짝수이다(필수!!) 따라서 아래 If문을 통해 짝수로 만들어 준다*/
    /*memcpy의 기준이 16bit 기준이기 때문에 size를 2로 나누어주어야 하기 떄문.. 즉 원래 size가 홀수이면 가상의 8bit를 만들어준다.*/
    if(size%2 == 1){
        size = size + 1;
    }

    int16* scibQueue = pSCIBTXqueue->SCI_pstQarr;
    Uint16 Spare_size = 0;

    if (pSCIBTXqueue->SCI_q_head + size >= pSCIBTXqueue->ref_8bit_Qsize){
        Spare_size = pSCIBTXqueue->ref_8bit_Qsize - pSCIBTXqueue->SCI_q_head;
        memcpy(scibQueue + (pSCIBTXqueue->SCI_q_head)/2, buf, Spare_size/2);
        buf = buf + Spare_size/2;
        size = size - Spare_size;
        pSCIBTXqueue->SCI_q_head = 0;
        memcpy(scibQueue + (pSCIBTXqueue->SCI_q_head)/2, buf, size/2);
        // 해당영역부터는 size가 SCI_Q_SIZE보다 클 때 별도의 처리알고리즘이 필요하지만 한번에 SCI_Q_SIZE를 넘는 데이터를 Enqueue할 일이 없기 때문에
        // 코드의 효율성을 위하여 작성하지 않았다.. 혹시나 필요할 경우 알아서 추가하도록 하자..
    }else{
        memcpy(scibQueue + (pSCIBTXqueue->SCI_q_head)/2, buf, size/2);
    }
    pSCIBTXqueue->SCI_q_head = pSCIBTXqueue->SCI_q_head + size;
}


void InitScic(void){

    float CPU_FREQ  = 200E6;
    float LSPCLK_FREQ = CPU_FREQ/2;
    float SCI_FREQ = 9600; //여기서 Buadrate 설정
    Uint16 SCI_PRD = (Uint16)(LSPCLK_FREQ/(SCI_FREQ*8))-1;

    ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.all =0x0003;

    ScicRegs.SCIHBAUD.bit.BAUD =(SCI_PRD >> 8);  // 9600 baud @LSPCLK = 100MHz
                                         //(200 MHz SYSCLK).
    ScicRegs.SCILBAUD.bit.BAUD =(SCI_PRD & 0x00FF);

    ScicRegs.SCIFFTX.all=0xC000;
    ScicRegs.SCIFFRX.all=0x0030;
    ScicRegs.SCIFFCT.all=0x0;

    ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

void InitScicQueue(void){

    Uint16 TXQueSZ = {SCIC_TXQUE/2};
    Uint16 RXQueSZ = {SCIC_RXQUE/2};
    Uint16 RXDecodeQueSZ = {SCIC_DecodeQUE};

    pSCICTXqueue = &SCICTXqueue;
    memset(&SCICTXqueue, 0x0, sizeof(SCICTXqueue));
    SCICTXqueue.SCI_Q_SIZE = TXQueSZ;
    pSCICTXqueue->ref_8bit_Qsize = SCIC_TXQUE;

    pSCICTXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * TXQueSZ);

    pSCICRXqueue = &SCICRXqueue;
    memset(&SCICRXqueue, 0x0, sizeof(SCICRXqueue));
    SCICRXqueue.SCI_Q_SIZE = RXQueSZ;
    pSCICRXqueue->ref_8bit_Qsize = SCIC_RXQUE;

    pSCICRXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * RXQueSZ);

    pSCICRXqueue->Decode_Buffer = (int16*)malloc(sizeof(int16) * RXDecodeQueSZ);
}

void ScicTxQueueFree(void){

    free(pSCICTXqueue->SCI_pstQarr);
}


void InitScid(void){

    Uint32 CPU_FREQ  = 200E6;
    Uint32 LSPCLK_FREQ = CPU_FREQ/2;

#if defined(NOVATEL_OEM_719)
    Uint32 SCI_FREQ = 460800; //여기서 Buadrate 설정
    Uint16 SCI_PRD = (Uint16)(LSPCLK_FREQ/(SCI_FREQ*8))-1;
#elif defined(UBLOX_NEO_8)
    Uint32 SCI_FREQ = 38400; //여기서 Buadrate 설정
    Uint16 SCI_PRD = (Uint16)(LSPCLK_FREQ/(SCI_FREQ*8))-1;
#endif

    ScidRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScidRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScidRegs.SCICTL2.all =0x0003;

    ScidRegs.SCIHBAUD.bit.BAUD =(SCI_PRD >> 8);  // 9600 baud @LSPCLK = 100MHz
                                         //(200 MHz SYSCLK).
    ScidRegs.SCILBAUD.bit.BAUD =(SCI_PRD & 0x00FF);

    ScidRegs.SCIFFTX.all=0xC000;
    ScidRegs.SCIFFRX.all=0x0030;

    ScidRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScidRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScidRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

void InitScidQueue(void){

    Uint16 TXQueSZ = {SCID_TXQUE/2};
    Uint16 RXQueSZ = {SCID_RXQUE/2};
    Uint16 RXDecodeQueSZ = {SCID_DecodeQUE};

    pSCIDTXqueue = &SCIDTXqueue;
    memset(&SCIDTXqueue, 0x0, sizeof(SCIDTXqueue));
    SCIDTXqueue.SCI_Q_SIZE = TXQueSZ;
    pSCIDTXqueue->ref_8bit_Qsize = SCID_TXQUE;

    pSCIDTXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * TXQueSZ);

    pSCIDRXqueue = &SCIDRXqueue;
    memset(&SCIDRXqueue, 0x0, sizeof(SCIDRXqueue));
    SCIDRXqueue.SCI_Q_SIZE = RXQueSZ;
    pSCIDRXqueue->ref_8bit_Qsize = SCID_RXQUE;

    pSCIDRXqueue->SCI_pstQarr = (int16*)malloc(sizeof(int16) * RXQueSZ);

    pSCIDRXqueue->Decode_Buffer = (int16*)malloc(sizeof(int16) * RXDecodeQueSZ);
}

void ScidTxQueueFree(void){

    free(pSCIDTXqueue->SCI_pstQarr);
}

void ScidTxEnQueue(int16 *buf, Uint16 size){
    /*여기서 입력값으로 받는 size는 8bit 기준이다*/
    /*버퍼의 크기(size)는 항상 짝수이다(필수!!) 따라서 아래 If문을 통해 짝수로 만들어 준다*/
    /*memcpy의 기준이 16bit 기준이기 때문에 size를 2로 나누어주어야 하기 떄문.. 즉 원래 size가 홀수이면 가상의 8bit를 추가해준다.*/
    if(size%2 == 1){
        size = size + 1;
    }
    int16* scidQueue = pSCIDTXqueue->SCI_pstQarr;
    Uint16 Spare_size = 0;

    if (pSCIDTXqueue->SCI_q_head + size >= pSCIDTXqueue->ref_8bit_Qsize){
        Spare_size = pSCIDTXqueue->ref_8bit_Qsize - pSCIDTXqueue->SCI_q_head;
        memcpy(scidQueue + (pSCIDTXqueue->SCI_q_head)/2, buf, Spare_size/2);
        buf = buf + Spare_size/2;
        size = size - Spare_size;
        pSCIDTXqueue->SCI_q_head = 0;
        memcpy(scidQueue, buf, size/2);
        // 해당영역부터는 size가 SCI_Q_SIZE보다 클 때 별도의 처리알고리즘이 필요하지만 한번에 SCI_Q_SIZE를 넘는 데이터를 Enqueue할 일이 없기 때문에
        // 코드의 효율성을 위하여 작성하지 않았다.. 혹시나 필요할 경우 알아서 추가하도록 하자..
    }else{
        memcpy(scidQueue + (pSCIDTXqueue->SCI_q_head)/2, buf, size/2);
    }
    pSCIDTXqueue->SCI_q_head = pSCIDTXqueue->SCI_q_head + size;
    ScidRegs.SCIFFTX.bit.TXFFIENA = 1;
}

#ifdef UBLOX_NEO_8

void Ublox_8_DEQUEUE(){

    Uint16 value = 0;

    value = (Uint16)__byte(pSCIDRXqueue->SCI_pstQarr, pSCIDRXqueue->SCI_q_tail++);
    pSCIDRXqueue->SCI_q_tail %= pSCIDRXqueue->ref_8bit_Qsize;

    switch(pSCIDRXqueue->Decode_Flag){

    //case UBX_H1,H2 are Ublox UBX packet Header
    case UBX_H1 :
        if(value == 0xB5)            (pSCIDRXqueue->Decode_Flag++);
        else pSCIDRXqueue->Decode_Flag = 0;
        break;

    case UBX_H2 :
        if(value == 0x62)            (pSCIDRXqueue->Decode_Flag++);
        else pSCIDRXqueue->Decode_Flag = 0;
        break;

    case UBX_CL : //UBX Class.
        pSCIDRXqueue->Class = value;
        pSCIDRXqueue->Decode_Flag++;
        break;

    case UBX_ID : //UBX ID.
        pSCIDRXqueue->ID = value;
        pSCIDRXqueue->Decode_Flag++;
        //HEADER CLASS ID 식별된 상황에서 시간 측정
        GET_time0();
        DEF_T0_END;
        MCU_TIME = T0CNT1;
        break;

    case UBX_SZ : //Msg. Length Low
        pSCIDRXqueue->Decode_Size = value;
        pSCIDRXqueue->Decode_Flag++;
        break;

    case (UBX_SZ+1) : //Msg. Length Total
        pSCIDRXqueue->Decode_Size = (Uint16) ((value << 8) | (pSCIDRXqueue->Decode_Size));
        pSCIDRXqueue->Decode_Flag++;
        break;

    case (UBX_PL) : //Msg. Length Total
        __byte(pSCIDRXqueue->Decode_Buffer, pSCIDRXqueue->Buff_index++) = value;
        if(pSCIDRXqueue->Buff_index > pSCIDRXqueue->Decode_Size + 1){ //get the checksum so size+1
        pSCIDRXqueue->Decode_Flag++;
        pSCIDRXqueue->Buff_index = 0;
        }
        break;

    default:
        break;
    }

    //Decode Part... 아래에서 식별한 class와 ID를 받아서 Decode 시작!!
    if(pSCIDRXqueue->Decode_Flag == (UBX_PL+1)){

        if( (pSCIDRXqueue->Class == 0x01) && (pSCIDRXqueue->ID == 0x07) ){ // Ublox PVT Packet

            NAV_PVT_Decode(pSCIDRXqueue->Decode_Buffer, MCU_TIME, pSCIDRXqueue->Decode_Size);
            pSCIDRXqueue->Decode_Flag = 0;

        }else if( (pSCIDRXqueue->Class == 0x01) && (pSCIDRXqueue->ID == 0x35) ){ //Ublox SAT Packet

            NAV_SAT_Decode(pSCIDRXqueue->Decode_Buffer, MCU_TIME, pSCIDRXqueue->Decode_Size);
            pSCIDRXqueue->Decode_Flag = 0;

        }else if(0){
            //추가 Packet 있을 시 마다 작성
        }else{
            pSCIDRXqueue->Decode_Flag = 0;
        }
    }
}
#endif

#ifdef NOVATEL_OEM_719
void NOVATEL719_DEQUEUE(){

    Uint16 value = 0;
    value = (Uint16)__byte(pSCIDRXqueue->SCI_pstQarr, pSCIDRXqueue->SCI_q_tail++);
    pSCIDRXqueue->SCI_q_tail %= pSCIDRXqueue->ref_8bit_Qsize;
    __byte(pSCIDRXqueue->Decode_Buffer, pSCIDRXqueue->Buff_index++) = value;

    switch(pSCIDRXqueue->Decode_Flag){

    //case Novatel Binary Packet header
    case Nov_H1 : //0
        if(value == 0xAA) {
            pSCIDRXqueue->Decode_Flag++;
        }
        else {
            pSCIDRXqueue->Decode_Flag = 0;
            pSCIDRXqueue->Buff_index = 0;
        }
        break;

    case Nov_H2 : //1
        if(value == 0x44) {
            pSCIDRXqueue->Decode_Flag++;
        }
        else {
            pSCIDRXqueue->Decode_Flag = 0;
            pSCIDRXqueue->Buff_index = 0;
        }
        break;

    case Nov_H3 : //2
        if(value == 0x12) {
            pSCIDRXqueue->Decode_Flag++;
        }
        else {
            pSCIDRXqueue->Decode_Flag = 0;
            pSCIDRXqueue->Buff_index = 0;
        }
        break;

    case Nov_Hlen : //3
        pSCIDRXqueue->Head_sz = value;
        pSCIDRXqueue->Decode_Flag++;
        break;

    case Nov_ID : //4
        pSCIDRXqueue->ID = value;
        pSCIDRXqueue->Decode_Flag++;
        break;

    case Nov_ID+1 : //5
        pSCIDRXqueue->ID = (Uint16) ((value << 8) | (pSCIDRXqueue->ID));
        pSCIDRXqueue->Decode_Flag++;
        //ID 확인된 상황에서 TIMECHECK
        GET_time0();
        DEF_T0_END;
        MCU_TIME = T0CNT1;
        break;

    case Nov_SZ :  //8
        pSCIDRXqueue->Decode_Size = value;
        pSCIDRXqueue->Decode_Flag++;
        break;

    case Nov_SZ+1 :  //9
        pSCIDRXqueue->Decode_Size = (Uint16) ((value << 8) | (pSCIDRXqueue->Decode_Size));
        pSCIDRXqueue->Decode_Flag++;
        break;

    case (Nov_PL) : //10
        pSCIDRXqueue->Decode_Flag++;
        break;

    default :  (pSCIDRXqueue->Decode_Flag++); break;

    case 10000 : //for safety
        pSCIDRXqueue->Decode_Flag = 0;
        pSCIDRXqueue->Buff_index = 0;
        break;
    }

    //Decode Part... 아래에서 식별한 ID를 받아서 Decode 시작!!
    if( (pSCIDRXqueue->Decode_Flag > (Nov_PL))
            && (pSCIDRXqueue->Buff_index > pSCIDRXqueue->Head_sz + pSCIDRXqueue->Decode_Size + 3) ){

        if( (pSCIDRXqueue->ID == 0x00F1) ){ // BESTXYZ

            BESTXYZ_Decode(pSCIDRXqueue->Decode_Buffer, MCU_TIME, pSCIDRXqueue->Head_sz, pSCIDRXqueue->Decode_Size);
            pSCIDRXqueue->Decode_Flag = 0;
            pSCIDRXqueue->Buff_index = 0;

        }else if(  (pSCIDRXqueue->ID == 0x002B) ){ //RANGE

            RANGE_Decode(pSCIDRXqueue->Decode_Buffer, MCU_TIME, pSCIDRXqueue->Head_sz, pSCIDRXqueue->Decode_Size);
            pSCIDRXqueue->Decode_Flag = 0;
            pSCIDRXqueue->Buff_index = 0;

        }else if( (pSCIDRXqueue->ID == 0x05AB) ){ //SATXYZ2
            pSCIDRXqueue->Decode_Flag = 0;
            pSCIDRXqueue->Buff_index = 0;

        }else{
            pSCIDRXqueue->Decode_Flag = 0;
            pSCIDRXqueue->Buff_index = 0;
        }
    }
}
#endif

/* SCI Interrupt Function */
__interrupt void scia_RX_ISR(void)
{
    //Enqueue Here
}

__interrupt void scia_TX_ISR(void)
{
    //Dequeue Here
}

__interrupt void scib_RX_ISR(void)
{
    //Enqueue Here
    Uint16 FF_size = ScibRegs.SCIFFRX.bit.RXFFST;
    while(FF_size != 0){
        __byte(pSCIBRXqueue->SCI_pstQarr, pSCIBRXqueue->SCI_q_head++) = ScibRegs.SCIRXBUF.all;
        pSCIBRXqueue->SCI_q_head %= pSCIBRXqueue->ref_8bit_Qsize;
        FF_size--;
    }
    ScibRegs.SCIFFRX.bit.RXFFIL = 16;
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

__interrupt void scib_TX_ISR(void)
{
    //Dequeue Here
    if(pSCIBTXqueue->SCI_q_head == pSCIBTXqueue->SCI_q_tail){
        ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
    }else{
        Uint16 FF_size = MAX_FIFO_size;
        while(FF_size--){
            if (pSCIBTXqueue->SCI_q_head == pSCIBTXqueue->SCI_q_tail) break;
            ScibRegs.SCITXBUF.all = __byte(pSCIBTXqueue->SCI_pstQarr, pSCIBTXqueue->SCI_q_tail++);
            pSCIBTXqueue->SCI_q_tail %= pSCIBTXqueue->ref_8bit_Qsize;
        }
    }
    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

__interrupt void scic_RX_ISR(void)
{
    //Enqueue Here
}

__interrupt void scic_TX_ISR(void)
{
    //Dequeue Here
}
__interrupt void scid_RX_ISR(void)
{
    //Enqueue Here
    Uint16 FF_size = ScidRegs.SCIFFRX.bit.RXFFST;
    while(FF_size != 0){
        __byte(pSCIDRXqueue->SCI_pstQarr, pSCIDRXqueue->SCI_q_head++) = ScidRegs.SCIRXBUF.all;
        pSCIDRXqueue->SCI_q_head %= pSCIDRXqueue->ref_8bit_Qsize;
        FF_size--;
    }
    ScidRegs.SCIFFRX.bit.RXFFIL = 16;
    ScidRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScidRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x80;       // Issue PIE ACK
}

__interrupt void scid_TX_ISR(void)
{
    //Dequeue Here
    if(pSCIDTXqueue->SCI_q_head == pSCIDTXqueue->SCI_q_tail){
        ScidRegs.SCIFFTX.bit.TXFFIENA = 0;
    }else{
        Uint16 FF_size = MAX_FIFO_size;
        while(FF_size--){
            if (pSCIDTXqueue->SCI_q_head == pSCIDTXqueue->SCI_q_tail) break;
            ScidRegs.SCITXBUF.all = __byte(pSCIDTXqueue->SCI_pstQarr, pSCIDTXqueue->SCI_q_tail++);
            pSCIDTXqueue->SCI_q_tail %= pSCIDTXqueue->ref_8bit_Qsize;
        }
    }
    ScidRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x80;       // Issue PIE ACK
}


