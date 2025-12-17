#include "Tupa_parameters_cpu01.h"

// Sinais da rede e do dc-link
typedef struct{
    float Vab;
    float Vbc;
    float Vca;
    float Va;
    float Vb;
    float Vc;
    float Vdc;
    float Ia;
    float Ib;
    float Ic;
    float Io;
}sSignal_red;

#define SIGNAL_RED_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0}
sSignal_red entradas_red = SIGNAL_RED_DEFAULTS;

//abc
typedef struct{
    float a;
    float b;
    float c;
} sABC;

#define ABC_DEFAULTS {0,0,0}
sABC Vabc = ABC_DEFAULTS;
sABC Iabc = ABC_DEFAULTS;
sABC Vabc_pwm = ABC_DEFAULTS;

// Alfa-beta
typedef struct{
    float alfa;
    float beta;
} sAlfaBeta;

#define ALFABETA_DEFAULTS {0,0}
sAlfaBeta Valfabeta = ALFABETA_DEFAULTS;
sAlfaBeta Ialfabeta = ALFABETA_DEFAULTS;
sAlfaBeta Valfabeta_pwm = ALFABETA_DEFAULTS;

//dq
typedef struct{
    float d;
    float q;
    float sindq;
    float cosdq;
} sDQ;

#define DQ_DEFAULTS {0,0,0,0}

//SOGI
typedef struct {
    float x;
    float y;
    float W;
    float b0;
    float b1;
    float b2;
    float b3;
    float a1;
    float a2;
    float V_sogi;
    float V_sogi1;
    float V_sogi2;
    float V_sogi_q;
    float V_sogi_q1;
    float V_sogi_q2;
    float Vm;
    float Vm1;
    float Vm2;
    float K_damp;
    float freq_res;
} sSOGI;

#define SOGI_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.01,60}
sSOGI SOG = SOGI_DEFAULTS;
sSOGI SOGB = SOGI_DEFAULTS;

//Controlador PI
typedef struct {
    int enab;
    float feedback;
    float setpoint;
    float error;
    float error_ant;
    float integr;
    float integr_ant;
    float Kp;
    float Ki;
    float output;
    float outMin;
    float outMax;
    float feedforward;
} sPI;

#define PI_PLL_GRID_DEFAULTS {0,0, 0, 0, 0, 0, 0, PI_PLL_GRID_KP, PI_PLL_GRID_KI,0,PI_PLL_GRID_OUTMIN , PI_PLL_GRID_OUTMAX, 0}
#define PI_Vdc_DEFAULTS {0, 0, 0, 0, 0, 0, 0, PI_Vdc_KP, PI_Vdc_KI, 0,PI_Vdc_OUTMIN , PI_Vdc_OUTMAX, 0}
#define PI_Q_DEFAULTS   {0, 0, 0, 0, 0, 0, 0, 0, PI_Q_KI, 0, PI_Q_OUTMIN , PI_Q_OUTMAX, 0}
sPI pi_Vdc = PI_Vdc_DEFAULTS;
sPI pi_Q = PI_Q_DEFAULTS;

//PLL
typedef struct {
    float amplitude;
    float freq;
    float omega;
    float omega_ant;
    float omega_init;
    float sinth;
    float costh;
    float theta;
    float theta_ant;
    float alfa;
    float beta;
    float Dpos;
    float Qpos;
    sPI PI_pll;
    sAlfaBeta alfabeta;
    sDQ dq;
} sSRFPLL;

#define SRFPLL_DEFAULTS {180,60,376.99,376.99,376.99,0,0,0,0,0,0,0,0,PI_PLL_GRID_DEFAULTS,ALFABETA_DEFAULTS,DQ_DEFAULTS}
sSRFPLL pll_grid = SRFPLL_DEFAULTS;

//PWM
typedef struct{
    Uint16 Ta;
    Uint16 Tb;
    Uint16 Tc;
} sSvm;

#define PWM_DEFAULTS {0,0,0}
sSvm sv_grid = PWM_DEFAULTS;

// Filtro de primeira ordem
typedef struct {
    float Un;
    float Un_1;
    float Yn_1;
    float Yn;
    float c0;
    float c1;
    } sFilter1st;
#define FILTER_DEFAULTS {0,0,0,0,0.00034906585039886593,0.00034906585039886593} //1Hz

sFilter1st Filt_freq_pll = FILTER_DEFAULTS;

// Média móvel
typedef struct {
    float array[N];
    float x;
    float y;
    float y_ant;
    int j;
    int Ns;
} sMAV;

#define  MAV_default {{0},0,0,0,0,N}
sMAV MAVQ = MAV_default;
sMAV MAVV = MAV_default;


typedef struct {
    float x;
    float x_ant;
    float x_ant2;
    float y;
    float y_ant;
    float y_ant2;
    double c0;
    double c1;
    double c2;
    double c3;
    double c4;
} sFilter2nd;

#define  FILTER2ND_DEFAULTS {0,0,0,0,0,0,0.00001212470404899192,0.00002424940809798384,0.00001212470404899192,-1.990128522794091,0.990177021610287}   //20Hz
sFilter2nd fil2nVdc = FILTER2ND_DEFAULTS;

// Flags
typedef struct {
    unsigned int GSC_PulsesOn;
    unsigned int BSC_PulsesOn;
    unsigned int Chopper_On;
    unsigned int Shutdown;
    unsigned int AbleToStart;
    unsigned int Inv_on;
    unsigned int real_time_buff;
    unsigned int precharge_ok;
    unsigned int precharge_fail;

} sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,1,0,0}
sFlags flag = FLAGS_DEFAULTS;

// Controlador Ressonante
typedef struct {
    double c1;
    double c2;
    double c3;
    double c4;
    float feedback;
    float setpoint;
    float error;
    float error_ant;
    float error_ant2;
    float res;
    float res_ant;
    float res_ant2;
    float Kp;
    float Ki;
    float output;
    float res_init;
    float feedforward;
    Uint16 enab;
    } sPR;

// Constantes padrão para a fundamental
#define PR_I_FUND_DEFAULTS {0.00002777574703951879,-0.00002777574703951879,-1.999561366949691,1.000000000000000,0,0,0,0,0,0,0,0,\
    PR_I_GRID_KP, PR_I_GRID_KI, 0, 0, 0, 0}

// Constantes padrão para o 5ª harmônico
#define PR_I_5_DEFAULTS {0.00002772703603807777,-0.00002772703603807777,-1.989043790736547,1.000000000000000,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

// Constantes padrão para o 7ª harmônico
#define PR_I_7_DEFAULTS {0.00002767837630659803,-0.00002767837630659803,-1.978544665925977,1.000000000000000,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

sPR PR_Ia_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ib_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ia_5 = PR_I_5_DEFAULTS;
sPR PR_Ib_5 = PR_I_5_DEFAULTS;
sPR PR_Ia_7 = PR_I_7_DEFAULTS;
sPR PR_Ib_7 = PR_I_7_DEFAULTS;

// Rampa
typedef struct {
    int   enab;
    float final;
    float final_ant;
    float atual;
    float in;
    float delta;
    int   flag;
    int   flag2;
    float range;
    float inc;
} Ramp;

#define VRamp_default {0,0,0,0,0,0,0,0,0.1,0.005}
#define QRamp_default {0,0,0,0,0,0,0,0,0.1,0.06}
Ramp VRamp = VRamp_default;
Ramp QRamp = QRamp_default;

// canais do ADC
typedef struct{
    int CH_1;
    int CH_2;
    int CH_3;
    int CH_4;
    int CH_5;
    int CH_6;
}sChannel_adc;

#define CHANNEL_DEFAULTS {0,0,0,0,0,0}
sChannel_adc channel_offset = CHANNEL_DEFAULTS;

// Contadores
typedef struct{
    Uint32 count1;
    Uint32 count2;
    Uint32 count3;
    Uint32 count4;
    Uint32 count5;
    Uint32 count6;
    Uint32 count7;
    Uint32 count8;
}counts;

#define COUNTS_DEFAULTS {0,0,0,0,0,0,0,0}
counts Counts = COUNTS_DEFAULTS;

// Váriaveis para enviar dados do CPU1 para o CPU2
typedef struct {
    unsigned int send0;
    float send1;
} SsendCPU1toCPU2;

#define SEND_DEFAULTS {0,0}
SsendCPU1toCPU2 Send = SEND_DEFAULTS;

// Váriaveis para receber dados do CPU1 para o CPU2
typedef struct {
    unsigned int *recv0;
    float *recv1;
} SrecvCPU2toCPU1;

#define RECV_DEFAULTS {0,0}
SrecvCPU2toCPU1 Recv = RECV_DEFAULTS;

// Status dos Contatores
typedef struct {
    Uint16 Q3s;
    Uint32 Time_cont_status; //tempo*(Freq de amostragem)
} Scontatores;

#define CONTATORES_DEFAULTS {0,3.0*Nsample*PWM_FREQ}
Scontatores Contat_status = CONTATORES_DEFAULTS;

// Variaveis para a comunicação SCI
typedef struct {
    float sci_out;
    Uint16 asci;
    bool decimal;
} Ssci;

#define SCI_DEFAULTS {0,0,false}
Ssci scia_p = SCI_DEFAULTS;
Ssci scia_q = SCI_DEFAULTS;
Ssci scia_soc = SCI_DEFAULTS;
Ssci scia_check1 = SCI_DEFAULTS;
Ssci scia_check2 = SCI_DEFAULTS;
Ssci scia_check3 = SCI_DEFAULTS;

// Variaveis de mensagem para a comunicação SCI
typedef struct {
    float pref;
    float qref;
    float socref;
    float check1;
    float check2;
    float check3;
    Uint16 soma_tx;
    Uint16 len_msg;
    int16 count;
    char msg_tx[len_sci];
    char msg_rx[len_sci];
    Uint16 sdata[8];    // Send data
    Uint16 rdata[8];    // Received data
}Ssci_mesg;

#define SCI_MSG_DEFAULTS {0,0,0,0,0,0,0,0,0,{0},{0},{0},{0}}
Ssci_mesg sci_msgA = SCI_MSG_DEFAULTS;

// Parâmetros do SCI  
volatile float pout = 0;
float qout = 0;
float soc = 0;
char reset[len_sci] = {0};
char aux[4] = {0, 0, 0, 0};
char aux2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
Uint16 reset_sci = 0;
int32 flag_tx = 0;
int flag_ena = 0;


// Funções 
void TxBufferAqu(Ssci_mesg *);
float RxBufferAqu(Ssci *, Ssci_mesg *);
int sumAscii(char *string, int len);

// Variaveis globais

// Ajuste do offset ??
float inv_nro_muestras = 0;

//Váriavel de teste ??
float  gn = 1;

//Buffers para plot
float32 AdcResults[RESULTS_BUFFER_SIZE];
float32 AdcResults2[RESULTS_BUFFER_SIZE];
float32 AdcResults3[RESULTS_BUFFER_SIZE];

//Variaveis de comunica��o entre o CPUs
int send = 0;

// V�ri�veis de Controle
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float  Iref = 0;        // Referência de corrente (sem malha externa)
float  Vdc_ref = 80;    // Tensão de referência da tensão do dc-link;
float  Q_ref   = 0;     // Referência de potência reativa;
float  Qm      = 0;     // Potência reativa medida

int selecao_plot = 0;
Uint16 fault = FAULT_OK;
Uint32 resultsIndex = 0;

//Variaveis para ajuste do offset
Uint32 first_scan = 1;
Uint32 sum_CH1 = 0; Uint32 sum_CH2 = 0; 
Uint32 sum_CH3 = 0; Uint32 sum_CH4 = 0; 
Uint32 sum_CH5 = 0; Uint32 sum_CH6 = 0;
Uint32 N_amostras = 60000;
