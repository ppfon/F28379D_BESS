//###########################################################################
//
// TITLE:  BESS

//
// Included Files
//
#include "F28x_Project.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Peripheral_Interruption_Setup_cpu01.h"
#include "Tupa_parameters_cpu01.h"
#include "estruturas.h"

//Main
void main(void)

{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio(); // Skipped for this example

// init the pins for the SCI ports.
//  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
//  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
// These functions are found in the F2837xD_Gpio.c file.
//
   GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
   GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
   GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);
//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;  // Disable the PIE

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

//
// Map ISR functions
//
    EALLOW;
    PieVectTable.IPC1_INT = &IPC1_INT; //function of the interruption of the IPC for communication of CPus
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;  // SCI Tx interruption
    PieVectTable.TIMER0_INT = &isr_cpu_timer0;  // SCI Tx interruption
    EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;       //ADC_B interrupt. Enables column 2 of the interruptions, page 79 of the workshop material
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1 SCIA_RX
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;   // Timer0
// Enable global Interrupts and higher priority real-time debug events:
//
    IER = M_INT1 | M_INT9; //Habilita a linha da tabela de interrup��o. correspondente ao ADC_B, pg 79 do material do workshop

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 50000);
    CpuTimer0Regs.TCR.all = 0x4001;

// Configure GPIOs
    GPIO_Configure();

// Configure Init SCI-A - fifo
    scia_fifo_init();
//
// Configure the ADC and power it up
//
    ConfigureADC();

// Configure DAC
    Setup_DAC();

//
// Configure the ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;         // Turn off the EPWM clock
    EDIS;

    ConfigureEPWM();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;        // Turn on the EPWM clock
    CpuSysRegs.PCLKCR0.bit.GTBCLKSYNC = 1;        // Turn on the Global clock
    EDIS;

//Transfere o Controle dos periféricos ADC e EPWM para o núcleo 2
    EALLOW;
    DevCfgRegs.CPUSEL0.bit.EPWM6 = 1;                   // Transfer ownership of EPWM6 to CPU2
    DevCfgRegs.CPUSEL0.bit.EPWM9 = 1;                   // Transfer ownership of EPWM6 to CPU2
    DevCfgRegs.CPUSEL0.bit.EPWM10 = 1;                   // Transfer ownership of EPWM6 to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_A = 1;                  // Transfer ownership of ADC_A to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_B = 1;                  // Transfer ownership of ADC_B to CPU2
    DevCfgRegs.CPUSEL11.bit.ADC_C = 1;                  // Transfer ownership of ADC_C to CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS8 = 1;                //Configura o Bloco GS8 da mem�ria RAM para o CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS9 = 1;                //Configura o Bloco GS9 da mem�ria RAM para o CPU2
    MemCfgRegs.GSxMSEL.bit.MSEL_GS10= 1;                //Configura o Bloco GS10 da mem�ria RAM para o CPU2
    EDIS;

    //
    //Habilita o CPU02 Carregar e Aguarda o seu carregamento carregamento atrav�s do loop finito
    ////Lembrete. Os IPCs que disparam interrup��o s�o o 0,1,2 e 3. Os outros n�o tem interrup��o e podem ser usados como flags
    //
    IpcRegs.IPCSET.bit.IPC5 = 1;                             //Seta o bit IPC5 para iniciar o carregamento do CPU02 carregar
    while (IpcRegs.IPCSTS.bit.IPC4 == 0);                    //Loop finito para aguardar o carregamento do CPU02
    IpcRegs.IPCACK.bit.IPC4 = 1;                             //Limpa a flag do IPC4

    // Habilita as GPIOs como ePwm
    InitEPwmGpio();

    // Ativa o Tipzone dos PWM e desabilita os pulsos at� o comando da flag.GSC_PulsesOn for habilitado
    EALLOW;
    EPwm1Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM1
    EPwm1Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
    EPwm1Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
    EPwm2Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM2
    EPwm2Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
    EPwm2Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
    EPwm5Regs.TZSEL.bit.OSHT1 = 0x1; // TZ1 configured for OSHT trip of ePWM5
    EPwm5Regs.TZCTL.bit.TZA = 0x2;   // Trip action set to force-low for output A
    EPwm5Regs.TZCTL.bit.TZB = 0x2;   // Trip action set to force-low for output B
    EDIS;

    //Habilita as Interrup��es. A partir desse ponto as interrup��es s�o chamadas quando requisitadas
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

// Initialize results buffer
    /*
    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
       AdcResults[resultsIndex] = 0;
       AdcResults2[resultsIndex] = 0;
       AdcResults3[resultsIndex] = 0;
    }
    */
    resultsIndex = 0;

    //Seta a variavel para ajuste do offset
    inv_nro_muestras = 1.0/N_amostras;

    //Habilita o controlador PI da PLL
    pll_grid.PI_pll.enab = 1;

    //Loop infinito
    while(1)
    {

    }
}

//Interrupção do IPC1 para comunicação com a CPU02
interrupt void IPC1_INT(void)
{
    Recv.recv0 = IpcRegs.IPCRECVADDR;
    IpcRegs.IPCACK.bit.IPC1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// sciaTxFifo - SCIA Transmit FIFO - ex: IA+9999F
interrupt void isr_cpu_timer0(void)
{
    Uint16 i;

    if (flag_ena == 1)
    {
       TxBufferAqu(&sci_msgA);

        for (i=0; i<len_sci; i++)
        {
            sci_msgA.sdata[i] = sci_msgA.msg_tx[i];
        }

        for(i=0; i < len_sci; i++)
        {
           SciaRegs.SCITXBUF.all=sci_msgA.sdata[i];  // Send data
        }

        SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;       // Issue PIE ACK
}

// sciaRxFifoIsr - SCIA Receive FIFO ISR
interrupt void sciaRxFifoIsr(void)
{
    Uint16 i;
    float pref_temp = 0;
    float qref_temp = 0;
    float soc_temp = 0;

    for(i=0;i<8;i++)
    {
        sci_msgA.rdata[i]= SciaRegs.SCIRXBUF.all;  // Read data
    }

    scia_p.asci = 65;
    scia_p.decimal = false;
    pref_temp = RxBufferAqu(&scia_p, &sci_msgA);

    sci_msgA.pref = pref_temp;

    scia_q.asci = 82;
    scia_q.decimal = false;
    qref_temp = RxBufferAqu(&scia_q, &sci_msgA);

    sci_msgA.qref = qref_temp;
    scia_soc.asci = 83;
    scia_soc.decimal = true;
    soc_temp = RxBufferAqu(&scia_soc, &sci_msgA);

    sci_msgA.socref = soc_temp;

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ack
}

// Tx funtion
void TxBufferAqu(Ssci_mesg *sci)
{
    Uint16 i = 0;
    Uint16 plen = 4;
    Uint16 qlen = 4;
    Uint16 soclen = 2;

    if (sci->count == 0)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "A");

        if ((int) pout >= 0) 
            strcat(sci->msg_tx, "+");
        else {
            if((int) pout < 0) strcat(sci->msg_tx, "-");
                else 
                    strcat(sci->msg_tx, "0");
        }

        sprintf(aux, "%d", (int) abs(pout));
        sci->len_msg = strlen(aux);
        if(sci->len_msg < plen)
        {
            for(i=0; i<(plen-sci->len_msg); i++)
                strcat(sci->msg_tx, "0");
        }
        strcat(sci->msg_tx, aux);

        if(pout > 9000) 
            pout = 0;

        strcat(sci->msg_tx, "\n");
    }

    if (sci->count == 1)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "R");

        if((int) qout >= 0) 
            strcat(sci->msg_tx, "+");
        else {
            if((int) qout < 0) 
                strcat(sci->msg_tx, "-");
            else 
                strcat(sci->msg_tx, "0");
        }

        sprintf(aux, "%d", (int) abs(qout));
        sci->len_msg = strlen(aux);
        if(sci->len_msg < qlen)
        {
            for(i=0; i<(qlen-sci->len_msg); i++)
                strcat(sci->msg_tx, "0");
        }
        strcat(sci->msg_tx, aux);

        strcat(sci->msg_tx, "\n");

    }

    if (sci->count == 2)
    {
        strcpy(sci->msg_tx, reset);

        strcat(sci->msg_tx, "I");

        strcat(sci->msg_tx, "S");

        int n_decimal_points_precision = 100;
        int integerPart = (int)soc;
        int decimalPart = ((int)(soc*n_decimal_points_precision)%n_decimal_points_precision);

        sprintf(aux, "%d", integerPart);
        sci->len_msg = strlen(aux);

        if(sci->len_msg < soclen)
        {
            for(i=0; i<(soclen-sci->len_msg); i++)
                strcat(sci->msg_tx, "0");
        }
        strcat(sci->msg_tx, aux);

        strcat(sci->msg_tx, ".");

        sprintf(aux, "%d", decimalPart);
        strcat(sci->msg_tx, aux);

        strcat(sci->msg_tx, "\n");

        sci->count = -1;

    }

    sci->count += 1;
}

// Rx funtion
float RxBufferAqu(Ssci *sci, Ssci_mesg *scimsg)
{
    Uint16 rstart = 0;
    Uint16 aq1 = 0;
    char aux_rx[5] = {0, 0, 0, 0, 0};
    Uint16 k = 0;
    Uint16 i = 0;
    Uint16 j = 0;

    while (1)
    {
        if (scimsg->rdata[i] == 73 && rstart == 0)
        {
            rstart = 1;
        }
        if(rstart == 1)
        {
            if(scimsg->rdata[i] == 10) break;

            if(aq1==1)
            {
                aux_rx[j] = (char) scimsg->rdata[i];
                j += 1;
            }
            if(scimsg->rdata[i] == sci->asci)
            {
                aq1 = 1;
                j = 0;
            }
        }

        i += 1;
        k += 1;

        if(i>=len_sci) i = 0;

        if(k>=16) break;
    }

    if (aq1 == 1 && sci->decimal == false) 
        sci->sci_out = strtol(aux_rx, NULL, 10);

    if (aq1 == 1 && sci->decimal == true) 
        sci->sci_out = strtof(aux_rx, NULL);

    return sci->sci_out;
}

int sumAscii(char *string, int len)
{
    int sum = 0;
    int j = 0;

    for (j = 0; j < len; j++)
        sum = sum + string[j];

    return sum;
}