/*
 * SCI_driver.h
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#ifndef INC_SCI_DRIVER_H_
#define INC_SCI_DRIVER_H_
/*****************************************************************************************************************/
/* 여기 설정된 QUEUE 사이즈는 8bit = 1byte 기준이다 */
/* DSP 변수는 최소크기가 16bit = 1byte이기 때문에 실제로는 아래 설정된 사이즈의 절반이 선언된다(따라서 짝수로 설정한다!!!)*/
/* 즉 선언된 크기가 3000이라면 DSP상에서 선언하는 Queue 사이즈는 1500이다(a[1500]이면 1500*16bit의 크기를 지니는 Queue 선언) */
/* 따라서 모든 head와 Tail은 8bit 접근자로 수행된다(중요) */
#define SCIA_TXQUE   100 //Not used
#define SCIA_RXQUE   100 //Not used
#define SCIB_TXQUE   500 //External Communication
#define SCIB_RXQUE   500 //External Communication
#define SCIC_TXQUE   100 //Not used
#define SCIC_RXQUE   100 //Not used


#if defined(NOVATEL_OEM_719)
#define SCID_TXQUE   200 //GNSS Novatel(TX will be free after command if wanted)
#define SCID_RXQUE   15000//GNSS Novatel
#elif defined(UBLOX_NEO_8)
#define SCID_TXQUE   200 //GNSS Ublox(TX will be free after command if wanted)
#define SCID_RXQUE   10000 //GNSS Ublox
#endif



#if defined(NOVATEL_OEM_719)
//여기의 버퍼는 16bit로 선언한다. head와 tail 사용X
#define SCIA_DecodeQUE   50
#define SCIB_DecodeQUE   50
#define SCIC_DecodeQUE   50
#define SCID_DecodeQUE   1200 //GNSS Ublox
#elif defined(UBLOX_NEO_8)
//여기의 버퍼는 16bit로 선언한다. head와 tail 사용X
#define SCIA_DecodeQUE   50
#define SCIB_DecodeQUE   50
#define SCIC_DecodeQUE   50
#define SCID_DecodeQUE   500 //GNSS Ublox
#endif
/*****************************************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////////SCI QUEUE DEFINITION
typedef struct {
    Uint16 SCI_Q_SIZE;
    Uint16 ref_8bit_Qsize;
    int16* SCI_pstQarr;
    Uint16 SCI_q_head;
    Uint16 SCI_q_tail;

    Uint16 Head_sz;
    Uint16 Decode_Flag;
    Uint16 Class;
    Uint16 ID;
    Uint16 Decode_Size;
    int16* Decode_Buffer;
    Uint16 Buff_index;
} structSCIQueue;

void InitScia(void);
void InitSciaQueue(void);
void SciaTxQueueFree(void);

void InitScib(void);
void InitScibQueue(void);
void ScibTxQueueFree(void);
void ScibTxEnQueue(int16 *buf, Uint16 size);

void InitScic(void);
void InitScicQueue(void);
void ScicTxQueueFree(void);

void InitScid(void);
void InitScidQueue(void);
void ScidTxQueueFree(void);
void ScidTxEnQueue(int16 *buf, Uint16 size);
void Ublox_8_DEQUEUE(void);
void NOVATEL719_DEQUEUE(void);
//*******************************Interrupt
__interrupt void scid_TX_ISR(void);
__interrupt void scid_RX_ISR(void);
__interrupt void scia_TX_ISR(void);
__interrupt void scia_RX_ISR(void);
__interrupt void scib_TX_ISR(void);
__interrupt void scib_RX_ISR(void);
__interrupt void scic_TX_ISR(void);
__interrupt void scic_RX_ISR(void);

#endif /* INC_SCI_DRIVER_H_ */
