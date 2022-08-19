//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "inc/Global_def.h"
#include "math.h"

//*******************************Timer Function Define
void GET_time0(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
//__interrupt void cpu_timer2_isr(void);

//*******************************DMA
__interrupt void INTCH5_ISR(void);
__interrupt void INTCH6_ISR(void);
//*******************************IMU_REG_SET(NOT Developed)
void IMU_REG_SET(void);
//*******************************GNSS RESCEIVER SETTING
void NaviModulesInit(void);
void (*pfGPS_Init)(void) = NULL;
void (*pfGNSS_SCI_DeQueue)(void) = NULL;

//*******************************Flag�� ���⿡
extern FlagStatus TIMER1_FLAG = RESET;
extern FlagStatus DMA_TX_DONE = RESET; //���� ������ ������ ���ܵ�
extern FlagStatus DMA_RX_DONE = SET; //�ʱ� Ʈ���� �߻�!!�� ���� SET
extern FlagStatus F_FileOpen = RESET; //SDī�� ���� ���� Flag
extern FlagStatus Q_data_write = RESET; //SDī�� �ʱ⼳�� �ܿ� �����;������� �� ���� Flag
extern FlagStatus IMU_READY = RESET; // IMU ������ ���� �غ� FLAG
extern FlagStatus System_Start = RESET; // ��� �ʱ⼳�� �� �ý��� ����(�������)
extern FlagStatus PVT_READY = RESET; // PVT���� ����
//*******************************Timer ���ú���
extern Uint32 Small_scale_timer0 = 0;
extern Uint32 Small_scale_timer1 = 0;
extern Uint32 T0CNT0 = 0;
extern Uint32 T0CNT1 = 0;
extern Uint32 T1CNT0 = 0;
extern Uint32 T1CNT1 = 0;
//*******************************SPI DMA ���ú���(IMU)
#pragma DATA_SECTION(rdata, "ramgs13");
#pragma DATA_SECTION(IMU_TIME, "ramgs13");
#pragma DATA_SECTION(sdata, "ramgs13");

volatile Uint16 *DMADest;
volatile Uint16 *DMASource;

Uint16 sdata[13];      // Send data buffer
Uint16 rdata[13];      // Receive data buffer
Uint32 IMU_TIME = 0;       // IMU TIME tag
Uint32 Time_Check = 0;       // IMU TIME tag
Uint32 Time_Check2 = 0;       // IMU TIME tag
Uint32 Time_Check3 = 0;       // IMU TIME tag

//*******************************SDī�� ���� ����
char pch[13] = {""};

//*******************************SCI QUEUE ���� ����
extern structSCIQueue      SCIATXqueue;
extern structSCIQueue*     pSCIATXqueue;

extern structSCIQueue      SCIARXqueue;
extern structSCIQueue*     pSCIARXqueue;

extern structSCIQueue      SCIBTXqueue;
extern structSCIQueue*     pSCIBTXqueue;

extern structSCIQueue      SCIBRXqueue;
extern structSCIQueue*     pSCIBRXqueue;

extern structSCIQueue      SCICTXqueue;
extern structSCIQueue*     pSCICTXqueue;

extern structSCIQueue      SCICRXqueue;
extern structSCIQueue*     pSCICRXqueue;

extern structSCIQueue      SCIDTXqueue;
extern structSCIQueue*     pSCIDTXqueue;

extern structSCIQueue      SCIDRXqueue;
extern structSCIQueue*     pSCIDRXqueue;

Uint16 i = 0;
Uint32 j = 0;
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
    DELAY_US(5000000); //Wait 10 seconds FOR Novatel Setup Time

#ifdef _STANDALONE
#ifdef _FLASH
    //
    // Send boot command to allow the CPU2 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);


#else
    //
    // Send boot command to allow the CPU2 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
#endif
#endif

    //
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    //
#ifdef _FLASH
    InitFlash();
#endif

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xD_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // GPIO CTRL ���� ������ ��� CPU1���� �ؾ��Ѵ�!!!
    InitGpio(); // Skipped for this example
//
//    ////////////////////////////////////////////////////////////////////////////////2.1 Two LED
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; // GPIO setting as Output.. �Ʒ� Function���� ��������
    GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 0);

    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1; // GPIO setting as Output.. �Ʒ� Function���� ��������
    GPIO_SetupPinOptions(11, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 0);

    GpioDataRegs.GPACLEAR.bit.GPIO10 =1;
    GpioDataRegs.GPACLEAR.bit.GPIO11 =1;
    EDIS;
    ////////////////////////////////////////////////////////////////////////////////2.2 For SCI
    //For UBlox ~~SCID
    EALLOW;
    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;  // Enable pull-up on GPIO22 (SCIRXDD)
    GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;  // Enable pull-up on GPIO23 (SCITXDD)

    GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;  // Enable pull-up on GPIO22 (SCIRXDD)
    GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;  // Enable pull-up on GPIO23 (SCITXDD)

    //
    //Configure SCI-D pins using GPIO regs
    //
    // This specifies which of the possible GPIO pins will be SCI functional
    // pins.
    // Comment out other unwanted lines.

    GpioCtrlRegs.GPBGMUX1.bit.GPIO46 = 1; // SCIRXDD mux
    GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 2;

    GpioCtrlRegs.GPBGMUX1.bit.GPIO47 = 1; // SCITXDD mux
    GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 2;
    EDIS;

    ////////////////////////////////////////////////////////////////////////////////2.3 For SPI
    ////// For IMU ~~SPIA
    EALLOW;
    //
    // Enable internal pull-up for the selected pins
    //
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
   GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;
   GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;

   //
   // Set qualification for selected pins to asynch only
   //
   // This will select asynch (no qualification) for the selected pins.
   // Comment out other unwanted lines.
   //
   GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;
   GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;
   GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;
   GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;


   //
   //Configure SPI-A pins using GPIO regs(������Ű�� �ʼ� GMUX->MUX)
   //
   GpioCtrlRegs.GPBGMUX2.bit.GPIO58 =3;
   GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;

   GpioCtrlRegs.GPBGMUX2.bit.GPIO59 =3;
   GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;

   GpioCtrlRegs.GPBGMUX2.bit.GPIO60 =3;
   GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;

   GpioCtrlRegs.GPBGMUX2.bit.GPIO61 =0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;

   GpioCtrlRegs.GPBDIR.bit.GPIO61 =1; //CS_PIN setting Output

   EDIS;

    //
    // TODO Add code to allow configuration of GPADIR from CPU02 using IPC
    //

    // Step 3.
    /////////////////////////
    //////////  SPI /////////
    /////////////////////////
    ///////////////////////////////////////////DMA�� ����ϱ� ���Ͽ� FIFO�� ���
    InitSpia();
//    InitSpic(); //SDī����� SPI�ʱ⼳���� TI �����ַ�� ���� �� ����, mmc-F2837x.c�� disk_initialize �Լ� ����


    // Step 4.
    /////////////////////////
    //////////  DMA /////////
    /////////////////////////
    ///////////////////////////////////////////DMA�� ������ Interrupt Trigger�� �����ϸ� ��κ� ���ü����� ���⼭ �Ѵ�.
    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1; //(0: CLA, 1: DMA)
    EDIS;

    dma_init();
    StartDMACH6();

    // Step 5.
    /////////////////////////
    //////////  SCI /////////
    /////////////////////////
    ///////////////////////////////////////////
    //~scid for GNSS
    InitScid();
    InitScidQueue();
    //~scib for Modem communication
    InitScib();
    InitScibQueue();

    // Step 6.
    /////////////////////////
    //////  Interrupt ///////
    /////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////���ͷ�Ʈ�� ��� �����Ͽ� ���⼭ �����Ѵ�.
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    //
    InitPieVectTable();

    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    //    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.DMA_CH5_INT= &INTCH5_ISR; //DMA_TX
    PieVectTable.DMA_CH6_INT= &INTCH6_ISR; //DMA_RX
    PieVectTable.SCID_RX_INT= &scid_RX_ISR; //SCID_RX
    PieVectTable.SCID_TX_INT= &scid_TX_ISR; //SCID_TX
//    PieVectTable.SCIA_RX_INT= &scia_RX_ISR; //SCIA_RX
//    PieVectTable.SCIA_TX_INT= &scia_TX_ISR; //SCIA_TX
    PieVectTable.SCIB_RX_INT= &scib_RX_ISR; //SCIB_RX
    PieVectTable.SCIB_TX_INT= &scib_TX_ISR; //SCIB_TX
//    PieVectTable.SCIC_RX_INT= &scic_RX_ISR; //SCIC_RX
//    PieVectTable.SCIC_TX_INT= &scic_TX_ISR; //SCIC_TX
    EDIS;

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, (Uint32)(TIM0COUNT+1)*10);
    ConfigCpuTimer(&CpuTimer1, 200, (Uint32)(TIM1COUNT+1)*10);
    //        ConfigCpuTimer(&CpuTimer2, 200, 1000000);

    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    //    CpuTimer2Regs.TCR.all = 0x4000;


    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER7.bit.INTx5 = 1;          // Enable PIE Group 7, INT 1 (DMA CH5)
    PieCtrlRegs.PIEIER7.bit.INTx6 = 1;          // Enable PIE Group 7, INT 2 (DMA CH6)
    PieCtrlRegs.PIEIER8.bit.INTx7 = 1;          // Enable PIE Group 8, INT 7 (SCID RX)
    PieCtrlRegs.PIEIER8.bit.INTx8 = 1;          // Enable PIE Group 8, INT 8 (SCID TX)
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;          // Enable PIE Group 9, INT 3 (SCIB RX)
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;          // Enable PIE Group 9, INT 4 (SCIB TX)


    IER |= M_INT1;
    IER |= M_INT7;                              // Enable PIE Group 7 ENABLE bit(FOR DMA)
    IER |= M_INT8;
    IER |= M_INT9;
    IER |= M_INT13;
    //    IER |= M_INT14;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    //////////////////////////////////////////////////////////////////////////////////LED All OFF
    LED1_OFF;
    LED2_OFF;

    //////////////////////////////////////////////////////////////////////////////////IMU DATA REQUEST PACKET SETTING
    TX_BufferAddress_ADIS16448(sdata);
    //////////////////////////////////////////////////////////////////////////////////SDCARD Initialization
    SD_Initialize(pch);
    Log_Init();
    //////////////////////////////////////////////////////////////////////////////////GNSS RECEIVER initialization
    NaviModulesInit();

    DEF_SETFLAG(System_Start);
    if(F_FileOpen && System_Start){
        LED1_ON;
    }

    while(1)        // wait until the DMA transfer is complete
    {
        //            asm(" ESTOP0");
        if(TIMER1_FLAG){
            LED2_OFF;
            j++;
            if(j%2500 == 0){
                SD_Sync();
                j = 0;
                LED2_ON;
            }
            DEF_RESETFLAG(TIMER1_FLAG);
        }

        /* SCI RX Interrupt(EnQUEUE)... FIFO�� ��� ä������ �ڵ����� ���ͷ�Ʈ�� �ɸ����� ä������ ���� ������(�����)ó���� ���½ð��� ȿ�������� Ȱ���ϱ� ���� �Ʒ��� ���� ������ ���� */
        /* �Ʒ��ڵ�� RXFFIL�� �����Ͽ� ������ ���ͷ�Ʈ�� �ɸ��� �ϴ� ���(���� ���ͷ�Ʈ �� �ȿ��� RXFFIL�� �ٽ� 16���� ������ ��)������ ���ͷ�Ʈ�� �������� �ٷ� �����͸� �޴� ����� �ִ�.. �ڵ� ���Ǽ��� ���� ���ͷ�Ʈ ������*/
        /* ��������� UART RX�� Interrupt�� Polling ����� �����ִٰ� �� �� �ִ� */
        if(ScibRegs.SCIFFRX.bit.RXFFST != 0){
            ScibRegs.SCIFFRX.bit.RXFFIL = ScibRegs.SCIFFRX.bit.RXFFST;
        }
        if(ScidRegs.SCIFFRX.bit.RXFFST != 0){
            ScidRegs.SCIFFRX.bit.RXFFIL = ScidRegs.SCIFFRX.bit.RXFFST;
        }

        /*************************************************SCI RX BUFFER DEQUEUE HERE***************************************************/
//        while(pSCIARXqueue->SCI_q_head != pSCIARXqueue->SCI_q_tail)   SciaRxDeQueue(); // Not used
//        while(pSCIBRXqueue->SCI_q_head != pSCIBRXqueue->SCI_q_tail)   ScibRxDeQueue(); // Not used Now(but for external Communication)
//        while(pSCICRXqueue->SCI_q_head != pSCICRXqueue->SCI_q_tail)   ScicRxDeQueue(); // Not used
        while(pSCIDRXqueue->SCI_q_head != pSCIDRXqueue->SCI_q_tail)   pfGNSS_SCI_DeQueue();
//
        QueueWrite();   //IMUDATA SDCARD WRITE
        Queue2Write();  //UBLOX PVT or NOVATEL BESTXYZ PACKET SDCARD WRITE
        Queue3Write();  //UBLOX SAT or NOVATEL RANGE PACKET SDCARD WRITE

    }
}

//�� �״�� �ȴ���... �ð��� ���� ������ 100us�� ���ش��� ������ �ð���!!(��� �͵� �� ���ͷ�Ʈ �켱������ �������)
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpu_timer1_isr(void)
{
    DEF_SETFLAG(TIMER1_FLAG);
    CpuTimer1.InterruptCount++;

    //Nesting interrupt part start
    IER |= M_INT7;
    EINT;

    if(DMA_RX_DONE){
        DEF_RESETFLAG(DMA_RX_DONE);
        //IMU REQUEST DMA TX
        CS_OFF_A;
        StartDMACH5();
        EALLOW;
        DmaRegs.CH5.CONTROL.bit.PERINTFRC = 1;
        EDIS;
        CS_ON_A;
    }
    //Nesting interrupt part finish
}
//
// local_D_INTCH5_ISR - DMA Channel 5 ISR
//
__interrupt void INTCH5_ISR(void)
{
    DEF_SETFLAG(DMA_TX_DONE);
    EALLOW;  // NEED TO EXECUTE EALLOW INSIDE ISR !!!
//    DmaRegs.CH5.CONTROL.bit.HALT=1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; // ACK to receive more interrupts
                                // from this PIE group
    EDIS;
    return;
}

//
// local_D_INTCH6_ISR - DMA Channel 6 ISR
//

__interrupt void INTCH6_ISR(void)
{
//    IMU TIME CHECK
    GET_time0();
    DEF_T0_END;
    IMU_TIME = T0CNT1;
    if(System_Start){
        UpdateIMUPacket(rdata, &IMU_TIME);
    }
    DEF_SETFLAG(DMA_RX_DONE);
    DEF_SETFLAG(IMU_READY);

    EALLOW;  // NEED TO EXECUTE EALLOW INSIDE ISR !!!
//    DmaRegs.CH6.CONTROL.bit.HALT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; // ACK to receive more interrupts
//                                             from this PIE group
    EDIS;
    return;
}


void dma_init()
{
    //
    // Initialize DMA
    //
    DMAInitialize();

    DMASource = (volatile Uint16 *)sdata;
    DMADest = (volatile Uint16 *)rdata;

    //
    // configure DMACH5 for TX
    //
    DMACH5AddrConfig(&SpiaRegs.SPITXBUF,DMASource);
    DMACH5BurstConfig(BURST,1,0);         // Burst size, src step, dest step
    DMACH5TransferConfig(TRANSFER,1,0);   // transfer size, src step, dest step
    DMACH5ModeConfig(DMA_SPIATX,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,
                     SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                     CHINT_END,CHINT_ENABLE);

    //
    // configure DMA CH6 for RX
    //
    DMACH6AddrConfig(DMADest,&SpiaRegs.SPIRXBUF);
    DMACH6BurstConfig(BURST,0,1);
    DMACH6TransferConfig(TRANSFER,0,1);
    DMACH6ModeConfig(DMA_SPIARX,PERINT_ENABLE,ONESHOT_DISABLE,CONT_ENABLE,
                     SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,
                     CHINT_END,CHINT_ENABLE);
}

// �Ʒ� �Լ��� Timer prescale�� ����� �� ����� ������ �����ϱ� ���� �ۼ��Ǿ���.
// TIM�� 0�� �� ������ Prescale Value�� 0�� �ƴϱ� ������ Timer Interrupt ���� �ٷ� 999999...(������ ���� �ٸ�)�� ���� ���Ѵ�.
// �̴� Timer�� ���ش��� �־ ����� ������ TIM�� 0�� ���� ������ 9999�� �ٻ�ȭ�Ѵ�.
// �� ����� ������ �ſ� ���� Ȯ���̱� ������ Timecheck�� ���� �� �ǵ�ġ �ʰ� �ð��� �����Ѱ�ó�� ���� �� �ִ�.
extern void GET_time0(void){
    Get_tim0;
    if(Small_scale_timer0 == 0){
        Small_scale_timer0 = 99999;
    }
}

void IMU_REG_SET(void)
{
}

void NaviModulesInit(void){

// * GPS Module * //

#if defined(NOVATEL_OEM_719)
    pfGPS_Init = NOVATEL719_Init;
    pfGNSS_SCI_DeQueue = NOVATEL719_DEQUEUE;
#elif defined(UBLOX_NEO_8)
    pfGPS_Init = NEO_Init;
    pfGNSS_SCI_DeQueue = Ublox_8_DEQUEUE;
#else
    // Send messages about not definition of a certain GPS module

#endif

    //Initialize GPS Environment
    pfGPS_Init();

}
//
// End of file
//
