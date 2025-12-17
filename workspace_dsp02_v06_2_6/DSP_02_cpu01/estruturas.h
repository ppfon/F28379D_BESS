 // Strutures //
// Dc-link and grid signals
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

//Alfa-beta
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

#define SOGI_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,60}
sSOGI SOG = SOGI_DEFAULTS;
sSOGI SOGB = SOGI_DEFAULTS;

//PI Controller
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
#define PI_Droop_DEFAULTS {0, 0, 0, 0, 0, 0, 0,PI_Droop_KP,PI_Droop_KI,0,PI_Droop_OUTMIN,PI_Droop_OUTMAX,0}
sPI pi_Vdc = PI_Vdc_DEFAULTS;
sPI pi_Q = PI_Q_DEFAULTS;
sPI pi_Droop = PI_Droop_DEFAULTS;

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

//Firs order LPF
typedef struct {
    float Un;
    float Un_1;
    float Yn_1;
    float Yn;
    float c0;
    float c1;
    } sFilter1st;
#define FILTER_DEFAULTS {0,0,0,0,0.00069813170079773186,0.00069813170079773186}
#define FILTER_DEFAULTS_2_5_HZ {0,0,0,0,0.00174532925199432959,0.00174532925199432959}
#define FILTER_DEFAULTS_1_HZ {0,0,0,0,0.00069813170079773186,0.00069813170079773186}

sFilter1st Filt_freq_pll = FILTER_DEFAULTS;
sFilter1st Filt_freq_Vdc = FILTER_DEFAULTS_1_HZ;
sFilter1st Filt_freq_Q = FILTER_DEFAULTS_1_HZ;
sFilter1st Filt_freq_Vpu = FILTER_DEFAULTS_1_HZ;

typedef struct{
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

#define  FILTER2ND_DEFAULTS_5Hz {0,0,0,0,0,0,0.00000303866583164036,0.00000607733166328071,0.00000303866583164036,-1.99506422020287144115,0.99507637486619804346}   //5Hz
#define  FILTER2ND_DEFAULTS_10Hz {0,0,0,0,0,0,0.00001212470404899192,0.00002424940809798384,0.00001212470404899192,-1.99012852279409080403,0.99017702161028686714}   //10Hz
sFilter2nd fil2nQ   = FILTER2ND_DEFAULTS_10Hz;
sFilter2nd fil2nP   = FILTER2ND_DEFAULTS_10Hz;
//sFilter2nd fil2nVpu   = FILTER2ND_DEFAULTS_10Hz;

//Flags
typedef struct{
    unsigned int GSC_PulsesOn;
    unsigned int BSC_PulsesOn;
    unsigned int Chopper_On;
    unsigned int Shutdown;
    unsigned int AbleToStart;
    unsigned int Inv_on;
    unsigned int real_time_buff;
    unsigned int precharge_ok;
    unsigned int precharge_fail;
    unsigned int manual_pre_charge;
    unsigned int Com_DSP2_read;
    unsigned int data_logo_init;
    unsigned int case_study;

}sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,1,0,0,0,0,0,0}
sFlags flag = FLAGS_DEFAULTS;

//Ressonante
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

#define PR_I_FUND_DEFAULTS {0.00005553931071838902,-0.00005553931071838902,-1.99824566019771654446,0.99999999999999977796,0,0,0,0,0,0,0,0,\
    PR_I_GRID_KP, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_5_DEFAULTS {0.00005515028886706705,-0.00005515028886706705,-1.95629520146761159971,1.00000000000000022204,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_7_DEFAULTS {0.00005476290380291145,-0.00005476290380291145,-1.91463899506413470775,1.00000000000000022204,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

#define PR_I_11_DEFAULTS {0.00005361052018169078,-0.00005361052018169078,-1.79142352047882602584,1.00000000000000044409,0,0,0,0,0,0,0,0,\
    0, PR_I_GRID_KI, 0, 0, 0, 0}

sPR PR_Ia_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ib_fund = PR_I_FUND_DEFAULTS;
sPR PR_Ia_5 = PR_I_5_DEFAULTS;
sPR PR_Ib_5 = PR_I_5_DEFAULTS;
sPR PR_Ia_7 = PR_I_7_DEFAULTS;
sPR PR_Ib_7 = PR_I_7_DEFAULTS;
sPR PR_Ia_11 = PR_I_11_DEFAULTS;
sPR PR_Ib_11 = PR_I_11_DEFAULTS;

//Ramp
typedef struct{
    float t1;
    float t1_ant;
    float y;
    float y_ant;
    float uin;
    float rate;
    float Ts;
    float rising;
    float falling;
} sRamp;
#define PRamp_default {0,0,0,0,0,0,TSAMPLE,9000,-9000}
#define VRamp_default {0,0,0,0,0,0,TSAMPLE,50,-50}
sRamp QRamp = PRamp_default;
sRamp VRamp = VRamp_default;


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

//Counters
typedef struct{
    Uint32 count1;
    Uint32 count2;
    Uint32 count3;
    Uint32 count4;
    Uint32 count5;
    Uint32 count6;
    Uint32 count7;
    Uint32 count8;
    Uint32 count9;
    Uint16 count10;
    Uint16 count11;
    Uint16 count_ipc;
}counts;

#define COUNTS_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0}
counts Counts = COUNTS_DEFAULTS;

//Variables to receive data from CPU1 to CPU2
typedef struct{
    unsigned int *recv0;
    float *recv1;
}SrecvCPU2toCPU1;

#define RECV_DEFAULTS {0,0}
SrecvCPU2toCPU1 Recv = RECV_DEFAULTS;

typedef struct{
    float sci_out;
    Uint16 asci;
    bool decimal;
}Ssci;

#define SCI_DEFAULTS {0,0,false}
Ssci scia_p = SCI_DEFAULTS;
Ssci scia_q = SCI_DEFAULTS;
Ssci scia_soc = SCI_DEFAULTS;
Ssci scia_check1 = SCI_DEFAULTS;
Ssci scia_check2 = SCI_DEFAULTS;
Ssci scia_check3 = SCI_DEFAULTS;

typedef struct{
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

// Estrutura para o droop
typedef struct{
float Vpu;
float Vcntrl;
float Gain_droop;
float Gain_droop1;
float Droop_max;
float Droop_min;
float Q_droop;
float aux;
float V_base;
}sCntrl_droop;

#define CNTRL_DROOP_DEFAULTS {0,0,GAIN_droop,GAIN_droop1,Q_Droop_OUTMAX,Q_Droop_OUTMIN,0,0,Vbase}
sCntrl_droop cntrl_droop = CNTRL_DROOP_DEFAULTS;

#define SCI_MSG_DEFAULTS {0,0,0,0,0,0,0,0,0,{0},{0},{0},{0}}
Ssci_mesg sci_msgA = SCI_MSG_DEFAULTS;
                                                                           // Functions //
void TUPA_abc2alfabeta(sABC *, sAlfaBeta *);
void TUPA_alfabeta2dq(sAlfaBeta*, sDQ* );
void TUPA_alfabeta2abc(sAlfaBeta* , sABC* );
void TUPA_SOGI(sSOGI *);
void TUPA_SRFPLL(sSRFPLL *);
void TUPA_Pifunc(sPI *);
void TUPA_pwm(sABC *, sSvm *, float, Uint16);
void TUPA_protect(void);
void TUPA_StartSequence(void);
void TUPA_StopSequence(void);
void Offset_Calculation(void);
void TUPA_First_order_signals_filter(sFilter1st *);
void TUPA_PR(sPR *);
void TUPA_Ramp(sRamp *);
void TUPA_Second_order_filter(sFilter2nd *);
void TUPA_Droop(sCntrl_droop *cd,float Qmed);              //função do Droop
void TxBufferAqu(Ssci_mesg *);
float RxBufferAqu(Ssci *, Ssci_mesg *);
int sumAscii(char *string, int len);
void sciaTxFifo(void);
                                                    // Global Variables //

//Variable for offset adjustment
float inv_nro_muestras = 0;

//Test variable
float  gn = 762;


//#pragma DATA_SECTION(gn,"SHARERAMGS1");         // Place the variable that will be shared with CPU02 in the following memory segment

//Buffers para plot
float32 AdcResults[RESULTS_BUFFER_SIZE];
float32 AdcResults2[RESULTS_BUFFER_SIZE];
float32 AdcResults3[RESULTS_BUFFER_SIZE];
float32 AdcResults4[RESULTS_BUFFER_SIZE];

//Buffers para armazenamento de dados
// float aqui_sign1[N_data_log];
// float aqui_sign2[N_data_log];
//float plota_dsp_2_cpu1[N_data_log];

//CPU comunication variables
int send = 0;

                                                              // Control Variables
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float  Iref = 0;
float  Vdc_ref = 480;
float  Q_ref   = 0;
float  Qm      = 0;
float  Pm      = 0;
float Q_control = 0;
float P_control = 0;
float Vref_droop = 1;  //tensão de referencia para o droop  1 PU
float Droop_enable = 0;  //variavel para ligar e desligar o Controle Droop


//Supervisor Variables
int selecao_plot = 0;
int contator_precharge = 0;
int contator_principal = 0;

Uint16 fault = FAULT_OK;
Uint32 resultsIndex = 0;
Uint32 resultsIndex2 = 0;
Uint32 resultsSampling = 0;
Uint32 SamplingBuf = 1;
//Variables for offset adjustment
Uint32 first_scan = 1;
Uint32 sum_CH1 = 0; Uint32 sum_CH2 = 0; Uint32 sum_CH3 = 0; Uint32 sum_CH4 = 0; Uint32 sum_CH5 = 0; Uint32 sum_CH6 = 0;
Uint32 N_amostras = 60000;

// SCI parameters
volatile float pout = 0;
float qout = 0;
float soc = 100;
char reset[len_sci] = {0};
char aux[4] = {0, 0, 0, 0};
char aux2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
Uint16 reset_sci = 0;
int32 flag_tx = 0;
int flag_ena = 0;




float Vabc_a=0;
float Vabc_b=0;
float Vabc_c=0;
float Valfabeta_alfa=0;
float Valfabeta_beta=0;
float VA=0;
float VB=0;
float freq_wn=0;
float Vabcp_a=0;
float Vabcp_b=0;
float Vabcp_c=0;
float Vabcn_a=0;
float Vabcn_b=0;
float Vabcn_c=0;
float Ki = 20*2*3.14159; // ki_pll
float Kp = 30*2*3.14159*30*2*3.14159; //ko_pll
float integOne_out =0;
float integOne_out_ant=0;
float integOne_in_ant=0;
float integOne_in=0;
float integTwo_out=0;
float integTwo_out_ant=0;
float integTwo_in_ant=0;
float integTwo_in=0;
float feedBack_Valfa=0;
float feedBack_Q_Valfa=0;
float V_alfa=0;
float Q_Valfa=0;
float V_beta=0;
float Q_Vbeta=0;
float feedBack_Vbeta=0;
float feedBack_Q_Vbeta=0;
float integOne_out_b=0;
float integOne_out_ant_b=0;
float integOne_in_ant_b=0;
float integOne_in_b=0;
float integTwo_out_b=0;
float integTwo_out_ant_b=0;
float integTwo_in_ant_b=0;
float integTwo_in_b=0;
float V_alfa_a;
float V_beta_a;
float V_alfa_b;
float V_beta_b;


//Angulos e Frequências PLL
float comp_d_a=0;
float comp_q_a=0;
float theta_a=0;
float Fnc_a=0;
float integOne_in_Fcn_a=0;
float integOne_out_Fcn_a=0;
float integOne_in_ant_Fcn_a=0;
float integOne_out_ant_Fcn_a=0;
float omega_a=0;

float integOne_in_Fcn_a1=0;
float integOne_out_Fcn_a1=0;
float integOne_out_ant_Fcn_a1=0;
float integOne_in_ant_Fcn_a1=0;

float comp_d_b=0;
float comp_q_b=0;
float theta_b=0;
float Fnc_b=0;
float integOne_in_Fcn_b=0;
float integOne_out_Fcn_b=0;
float integOne_in_ant_Fcn_b=0;
float integOne_out_ant_Fcn_b=0;
float omega_b=0;

float integOne_in_Fcn_b1=0;
float integOne_out_Fcn_b1=0;
float integOne_out_ant_Fcn_b1=0;
float integOne_in_ant_Fcn_b1=0;


// ESTRATÉGIAS DE CONTROLE - variáveis de calculos intermediários

float VabcP;
float VabcN;
float Vsoma;
float Vcorr;
float Pref;
//float Qref;
float divide;
float divide_1;
float prod_1x;
float prod_1y;
float prod_1z;
float Pcc_a = 0;
float Pcc_b = 0;
float Pcc_c = 0;
float Pcc_a_trans;
float Pcc_b_trans;
float Pcc_c_trans;
float Vabcp_a_trans;
float Vabcp_b_trans;
float Vabcp_c_trans;
float Vabcn_a_trans;
float Vabcn_b_trans;
float Vabcn_c_trans;
float prod_2x;
float prod_2y;
float prod_2z;
float soma_prod_x;
float soma_prod_y;
float soma_prod_z;
float I_a;
float I_b;
int teste01;
float teste02;

int selecao_stgy = 0;

// Case Study
int flag_case_study = 1;

