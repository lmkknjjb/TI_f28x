//###########################################################################
//
// FILE:   dma_transfer_shared_peripheral_cpu02.c
//
// TITLE:  DMA Transfer for Shared Peripheral Example for F2837xD.
//
// This example shows how to initiate a DMA transfer on CPU1 from a shared
// peripheral which is owned by CPU2.  In this specific example, a timer ISR
// is used on CPU2 to initiate a SPI transfer which will trigger the CPU1 DMA.
// CPU1's DMA will then in turn update the EPWM1 CMPA value for the PWM which
// it owns.  The PWM output can be observed on the GPIO pins configured in
// the InitEPwm1Gpio() function.
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
#include "F28x_Project.h"

//
// Pragma - Put the ePWM CMP value into shared RAM
//
#pragma DATA_SECTION(new_cmp_value, "SHARERAMGS2");

//
// Function Prototypes
//
void InitSpi(void);
void InitCpuTimer(void);
__interrupt void cpu_timer1_isr(void);
void load_buffer(void);

//
// Global variable used in this example
//
Uint16 new_cmp_value;
Uint16 direction = 1;

//
// Main
//
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Wait for verification from CPU1 that CPU2 has been given control
// of SPI-A and for control of GS2 to initialize the CMP value to a proper
// value. This is important to prevent glitching on the EPWM when the first
// DMA transfer occurs.
//
    while(IpcRegs.IPCSTS.bit.IPC0 == 0);

    new_cmp_value = 3000;  // Set CMP value

//
// Step 3. Turn on clock to SPI-A
//
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
    EDIS;

//
// Step 4. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Step 5. Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Step 6. Disable CPU interrupts and clear all CPU interrupt flags.
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Step 7. Initialize the PIE vector table with pointers to the shell
// Interrupt Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Step 8. Wait for IPC from CPU1 confirming DMA is configured before
// initializing SPI. Note that because of the way the TXFIFO interrupt
// is configured a DMA transfer will be triggered immediately after the
// SPI is released from reset
//
    while(IpcRegs.IPCSTS.bit.IPC1 == 0);

//
// Step 9. Setup SPI for FIFO mode
//
    InitSpi();

//
// Step 10. Setup CPU Timer 1 to interrupt every 10 ms
//
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer1, 200, 10000);
    CpuTimer1Regs.TCR.all = 0x4000;

//
// Step 11. Configure CPU Timer 1 ISR
//
    EALLOW;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    IER |= M_INT13;
    EDIS;

//
// Step 12. Enable global Interrupts and higher priority real-time debug
// events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Step 13. IDLE loop. Just sit and loop forever (optional):
//
    for(;;)
    {
        asm ("  NOP");
    }
}

//
// InitSpi - Function to Setup SPI for FIFO mode
//
void InitSpi(void)
{
   //
   // Initialize SPI FIFO registers
   //
   SpiaRegs.SPICCR.bit.SPISWRESET = 0; // Reset SPI
   SpiaRegs.SPICCR.all = 0x001F;       //16-bit character, Loopback mode
   SpiaRegs.SPICTL.all = 0x0017;       //Interrupt enabled, Master/Slave XMIT
                                       //enabled
   SpiaRegs.SPISTS.all = 0x0000;
   SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x0063;   // Baud rate
   SpiaRegs.SPIFFTX.all = 0xC022;               // Enable FIFO's, set TX FIFO
                                                // level to 4
   SpiaRegs.SPIFFRX.all = 0x0022;               // Set RX FIFO level to 4
   SpiaRegs.SPIFFCT.all = 0x00;
   SpiaRegs.SPIPRI.all = 0x0000;
   SpiaRegs.SPICCR.bit.SPISWRESET = 1;          // Enable SPI
   SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
   SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;

   //
   // A DMA transfer will be triggered here!
   //

   //
   // Load the SPI FIFO Tx Buffer
   //
   load_buffer();

   //
   // Disable the clock to prevent continuous transfer / DMA triggers
   // Note this method of disabling the clock should not be used if
   // actual data is being transmitted
   //
   EALLOW;
   CpuSysRegs.PCLKCR8.bit.SPI_A = 0;
   EDIS;
}

//
// cpu_timer1_isr - Function for CPU Timer1 Interrupt Service Routine
//
__interrupt void cpu_timer1_isr(void)
{
    //
    // Re-enable SPI clock to allow DMA trigger
    //
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
    EDIS;

    //
    // Wait for interrupt flag
    // This is when the DMA trigger will occur
    //
    while(!SpiaRegs.SPIFFTX.bit.TXFFINT);

    //
    // Reload the SPI TX buffer and clear interrupt flag
    //
    load_buffer();

    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

    //
    // Disable the clock to prevent continuous transfer / DMA triggers
    // Note this method of disabling the clock should not be used if
    // actual data is being transmitted
    //
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_A = 0;
    EDIS;

    //
    // Update next value to be transferred to the EPWM
    //
    if(new_cmp_value >= 4500)
    {
        direction = 0;
    }
    else if(new_cmp_value <= 1500)
    {
        direction = 1;
    }

    if(!direction)
    {
        new_cmp_value -= 50;
    }
    else
    {
        new_cmp_value += 50;
    }
}

//
// load_buffer - Function to load SPI Tx FIFO Buffer
//
void load_buffer(void)
{
    //
    // Fill up the FIFO buffer
    //
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
    SpiaRegs.SPITXBUF = 0xAAAA;
}

//
// End of file
//
