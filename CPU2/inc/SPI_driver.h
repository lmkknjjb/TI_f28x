/*
 * SPI_driver.h
 *
 *  Created on: 2018. 12. 17.
 *      Author: Administrator
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

////////////////////////////////////////////////////////////////////////////////////////////////DMA for ADIS16448
///�Ʒ� �� ������ TI REFERENCE�� ���ؼ� ����(SET, CLEAR ���)
#define CS_ON_A (GpioDataRegs.GPBCLEAR.bit.GPIO61 =1)
#define CS_OFF_A (GpioDataRegs.GPBSET.bit.GPIO61 =1)
///�Ʒ� ������ ���ӵ� ����� ���δٸ� GPIO�ɿ��� �̷���� �� ������ ���� �� ����!!(SPI CS������ ���ô� ���� �߻�����)
//#define CS_OFF_A (GpioDataRegs.GPBDAT.bit.GPIO61 =1)
//#define CS_ON_A (GpioDataRegs.GPBDAT.bit.GPIO61 =0)

#define ADIS16448_READ 0x0000 // ADIS16448 READMODE
#define ADIS16448_WRITE 0x8000 // ADIS16448 WRITEMODE
#define GLOB_CMD 0x3E00 // ADIS16448 CMD or BURST_MODE with READMODE

#define FIFO_LVL      13              // FIFO Interrupt Level
#define BURST         (FIFO_LVL-1)    // burst size should be less than FIFO_LVL
#define TRANSFER      0            // [(MEM_BUFFER_SIZE/FIFO_LVL)-1]

#define SPI_DATA_BUF_SIZE 20
#define FIFO_SIZE 16

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

#endif /* INC_SPI_DRIVER_H_ */
