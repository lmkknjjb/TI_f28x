/*
 * SPI_driver.h
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

////////////////////////////////////////////////////////////////////////////////////////////////For ADIS16448
///아래 에 설정은 TI REFERENCE에 준해서 만듬(SET, CLEAR 사용)
#define CS_ON_A (GpioDataRegs.GPBCLEAR.bit.GPIO61 =1)
#define CS_OFF_A (GpioDataRegs.GPBSET.bit.GPIO61 =1)
///아래 설정은 연속된 명령이 서로다른 GPIO핀에서 이루어질 때 문제가 생길 수 있음!!(SPI CS핀으로 사용시는 문제 발생안함)
//#define CS_OFF_A (GpioDataRegs.GPBDAT.bit.GPIO61 =1)
//#define CS_ON_A (GpioDataRegs.GPBDAT.bit.GPIO61 =0)
////////////////////////////////////////////////////////////////////////////////////////////////For SDCARD
#define CS_ON_C (GpioDataRegs.GPCCLEAR.bit.GPIO72 =1)
#define CS_OFF_C (GpioDataRegs.GPCSET.bit.GPIO72 =1)
////////////////////////////////////////////////////////////////////////////////////////////////ADIS16448 REGISTER
#define ADIS16448_READ 0x0000 // ADIS16448 READMODE
#define ADIS16448_WRITE 0x8000 // ADIS16448 WRITEMODE
#define GLOB_CMD 0x3E00 // ADIS16448 CMD or BURST_MODE with READMODE

////////////////////////////////////////////////////////////////////////////////////////////////DMA_SPI Value
#define FIFO_LVL      13              // FIFO Interrupt Level
#define BURST         (FIFO_LVL-1)    // burst size should be less than FIFO_LVL
#define TRANSFER      0            // [(MEM_BUFFER_SIZE/FIFO_LVL)-1]


typedef enum BUF_ADDR {

    CMD = 0,
    DIAG_STAT,
    XGYRO,
    YGYRO,
    ZGYRO,
    XACCL,
    YACCL,
    ZACCL,
    XMAGN,
    YMAGN,
    ZMAGN,
    BARO,
    TEMP,

} ADIS16448_Address;

void TX_BufferAddress_ADIS16448(Uint16* pBuff);
void UpdateIMUPacket(Uint16* IMUdata, Uint32* IMUTime);

#endif /* INC_SPI_DRIVER_H_ */
