//###########################################################################
//
// FILE:   F2837xD_Spi.c
//
// TITLE:  F2837xD SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#ifdef CPU1
#include "inc/SPI_driver.h"
#endif

//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//

//#if CPU_FRQ_200MHZ
//#define SPI_BRR        ((200E6 / 4) / 500E3) - 1 //500KHz
//#define SPI_BRR_1MHZ ((200E6 / 4) / 1E6) - 1 //1MHz
//#define SPI_BRR_10MHZ ((200E6 / 4) / 1E7) - 1 //10MHz
//#define SPI_BRR_20MHZ ((200E6 / 4) / 2E7) - 1 //10MHz
//#endif


// Edited
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 2  (edit)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1


#if CPU_FRQ_200MHZ
#define SPI_BRR_1MHZ (200E6/ 2 / 1E6) - 1 //1MHz
#define SPI_BRR_10MHZ (200E6/ 2 / 1E7) - 1 //10MHz
#define SPI_BRR_20MHZ (200E6/ 2 / 2E7) - 1 //10MHz
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
#endif

//
// InitSPI - This function initializes the SPI to a known state
//

#ifdef CPU1
void InitSpia(void)
{
        // Initialize SPI-A for

    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Enable loop-back
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;
    SpiaRegs.SPICCR.bit.SPICHAR = (16-1);
    SpiaRegs.SPICCR.bit.SPILBK = 0;

    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR_1MHZ; //100kHZ
    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpiaRegs.SPIPRI.bit.FREE = 0;
    SpiaRegs.SPIFFCT.bit.TXDLY=0;
    //FIFO REG
    SpiaRegs.SPIFFRX.all=0x2040;             // RX FIFO enabled, clear FIFO int
    SpiaRegs.SPIFFRX.bit.RXFFIL = FIFO_LVL;  // Set RX FIFO level

    SpiaRegs.SPIFFTX.all=0xE040;             // FIFOs enabled, TX FIFO released,
    SpiaRegs.SPIFFTX.bit.TXFFIL = FIFO_LVL;  // Set TX FIFO level

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

    SpiaRegs.SPICCR.bit.SPISWRESET = 1; // Release the SPI from reset

}

void InitSpic(void)
{
    // Initialize SPI-A

    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Enable loop-back
    // TI에서 제공하는 TIm LOVE의 SPRAAO7 설정을 따름
    SpicRegs.SPICCR.bit.SPISWRESET = 0;
    SpicRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpicRegs.SPICCR.bit.SPICHAR = (8-1);
    SpicRegs.SPICCR.bit.SPILBK = 0;

    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpicRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpicRegs.SPICTL.bit.TALK = 1;
    SpicRegs.SPICTL.bit.CLK_PHASE = 1;
    SpicRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpicRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR_20MHZ;

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpicRegs.SPIPRI.bit.FREE = 1;

    // Not Use FIFO MODE
//    SpicRegs.SPIFFRX.all=0x6040;             // RX FIFO enabled, clear FIFO int
//    SpicRegs.SPIFFRX.bit.RXFFIL = FIFO_LVL;  // Set RX FIFO level
//
//    SpicRegs.SPIFFTX.all=0xE040;             // FIFOs enabled, TX FIFO released,
//    SpicRegs.SPIFFTX.bit.TXFFIL = FIFO_LVL;  // Set TX FIFO level

    SpicRegs.SPICCR.bit.SPISWRESET = 1;    // Release the SPI from reset

}

//
//
// End of file
//
#endif
