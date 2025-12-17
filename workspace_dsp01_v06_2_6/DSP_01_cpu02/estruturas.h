
//Sinais dc
typedef struct{
    float I1;
    float I2;
    float I3;
    float Vb1;
    float Vb2;
    float Vb3;
    float Vbt;
    float Vbt_filt;
    float Vdc;
}sSignal_dc;

#define SIGNAL_DC_DEFAULTS {0,0,0,0,0,0,0,0,0}
sSignal_dc entradas_dc = SIGNAL_DC_DEFAULTS;

// Controlador PI
typedef struct {
    int enab;
    float feedback;
    float setpoint;
    float error;
    float error_ant;
    float errorpi;
    float errorpi_ant;
    float integr;
    float integr_ant;
    float dif;
    float Kp;
    float Ki;
    float output;
    float output_sat;
    float outMin;
    float outMax;
    float feedforward;
} sPI;

#define PI_IDIS_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_DIS_KP, PI_DIS_KI, 0, 0, PI_DIS_OUTMIN , PI_DIS_OUTMAX, 0}
#define PI_ICH_DEFAULTS  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_ICH_KP, PI_ICH_KI, 0, 0, PI_ICH_OUTMIN , PI_ICH_OUTMAX, 0}
#define PI_VCH_DEFAULTS  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PI_VCH_KP, PI_VCH_KI, 0, 0, 0, 0, 0}
sPI pi_I1_dis = PI_IDIS_DEFAULTS;
sPI pi_I2_dis = PI_IDIS_DEFAULTS;
sPI pi_I3_dis = PI_IDIS_DEFAULTS;
sPI pi_I1_ch  = PI_ICH_DEFAULTS;
sPI pi_I2_ch  = PI_ICH_DEFAULTS;
sPI pi_I3_ch  = PI_ICH_DEFAULTS;
sPI piv_ch    = PI_VCH_DEFAULTS;

//PWM
typedef struct{
    float din;
    Uint16 Ta;
} sPWM;

#define PWM_DEFAULTS {0,0}
sPWM pwm_dc = PWM_DEFAULTS;
sPWM pwm_dc2 = PWM_DEFAULTS;
sPWM pwm_dc3 = PWM_DEFAULTS;

//Flags
typedef struct{
    unsigned int BSC_PulsesOn;
    unsigned int Conv_on;
    unsigned int Shutdown_Conv;
    unsigned int Stand_by;
    unsigned int AbleToStart;
    unsigned int Bat_Charge;
    unsigned int Bat_Discharge;
    unsigned int Bat_Mode;
    unsigned int BVCM;
    unsigned int data_logo_init;
    unsigned int real_time_buff;
    unsigned int Com_DSP1_read;
}sFlags;

#define FLAGS_DEFAULTS {0,0,0,0,0,0,0,1,0,0,1,0}
sFlags flag = FLAGS_DEFAULTS;

//Rampa
typedef struct{
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

#define IRamp_default {0,0,0,0,0,0,0,0,0.01,0.008}
#define VRamp_default {0,0,0,0,0,0,0,0,0.01,0.01}
Ramp I1_Ramp = IRamp_default;
Ramp I2_Ramp = IRamp_default;
Ramp I3_Ramp = IRamp_default;
Ramp VRamp   = VRamp_default;

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

#define  FILTER2ND_50Hz_DEFAULTS {0,0,0,0,0,0,0.00029719248951703286,0.00059438497903406572,0.00029719248951703286,-1.95065639854559402799,0.95184516850366218677}
sFilter2nd fil2nVbat  = FILTER2ND_50Hz_DEFAULTS;
sFilter2nd fil2nVdc   = FILTER2ND_50Hz_DEFAULTS;
sFilter2nd fil2nVpu   = FILTER2ND_50Hz_DEFAULTS;

// Filtro passa-baixa de primeira ordem
typedef struct {
    float Un;
    float Un_1;
    float Yn_1;
    float Yn;
    float c0;
    float c1;
    } sFilter1st;
#define FILTER_DEFAULTS_0_1_HZ {0,0,0,0,0.00006981317007977319,0.00006981317007977319}

sFilter1st fil1nVbat = FILTER_DEFAULTS_0_1_HZ;
sFilter1st fil1nVbat2 = FILTER_DEFAULTS_0_1_HZ;
sFilter1st fil1nVbat3 = FILTER_DEFAULTS_0_1_HZ;

// Canais para retirar offset
typedef struct{
    int CH_1;
    int CH_2;
    int CH_3;
}sChannel_adc;

#define CHANNEL_DEFAULTS {0,0,0}
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
    Uint32 count9;
    Uint32 count10;
    Uint32 count11;
}counts;

#define COUNTS_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0}
counts Counts = COUNTS_DEFAULTS;

//Váriaveis para enviar dados do CPU2 para o CPU1
typedef struct{
    unsigned int send0;
    float send1;
}SsendCPU2toCPU1;

#define SEND_DEFAULTS {0,0}
SsendCPU2toCPU1 Send = SEND_DEFAULTS;

//Váriaveis para receber dados do CPU1 para o CPU2
typedef struct{
    unsigned int *recv0;
    float *recv1;
}SrecvCPU1toCPU2;

#define RECV_DEFAULTS {0,0}
SrecvCPU1toCPU2 Recv = RECV_DEFAULTS;

// Funções
// Controle
void TUPA_Pifunc(sPI *);
void TUPA_protect(void);
void Stand_by_mode(void);
void TUPA_StartSequence(void);
void TUPA_pwm(sPWM *, Uint16);
void TUPA_StopSequence(void);
void Offset_Calculation(void);
void TUPA_Ramp(Ramp *);
void TUPA_Second_order_filter(sFilter2nd *);
void TUPA_First_order_signals_filter(sFilter1st *);

// Variáveis globais 
//Variavel para ajuste do offset
float inv_nro_muestras = 0;

//Buffers para plot
// float32 AdcResults[RESULTS_BUFFER_SIZE];
// float32 AdcResults2[RESULTS_BUFFER_SIZE];
// float32 AdcResults3[RESULTS_BUFFER_SIZE];

//Buffers para armazenamento de dados
// float Vbat_vec[N_data_log] ;
// float Ibat_vec[N_data_log];
//float plota_dsp1_cpu2[N_data_log];

//Variaveis de comunicação entre o CPUs
int send = 0;

//V�riaveis de teste
float  gn   = 934.0;

// V�ri�veis de Controle
float Ts = TSAMPLE;
float Van = 0, Vbn = 0, Vcn = 0 , vmin = 0, vmax = 0, Vao = 0, Vbo = 0, Vco = 0;

float I_dis_ref   =  5;                         //Refer�ncia da corrente de descarga (modo Boost)
float I_ch_ref    =  5;                         //Refer�ncia da corrente de carga (modo Buck)
float Vboost      =  14.4;                      //Tens�o de Boost
float Vfloat      =  13.6;                      //Tens�o de Float
float Vref        =  0;                         //Refer�ncia da tens�o de carga

int selecao_plot = 0;
float Itotal = 0;

Uint16 fault = FAULT_OK;
Uint32 resultsIndex = 0;
Uint32 resultsIndex2 = 0;

//Variaveis para ajuste do offset
Uint16 first_scan = 1; Uint16 first_scan2 = 1; Uint16 first_scan3 = 1;
Uint32 sum_CH1 = 0; Uint32 sum_CH2 = 0; Uint32 sum_CH3 = 0;

Uint32 N_amostras = 60000;

