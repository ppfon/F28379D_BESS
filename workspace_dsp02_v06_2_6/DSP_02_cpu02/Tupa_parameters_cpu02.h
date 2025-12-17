#ifndef     __TUPA_PARAMETERS_H__
#define     __TUPA_PARAMETERS_H__

// PWM DEFINITIONS
#define PWM_FREQ                9000                        //Frequ�ncia de chaveamento
#define Nsample                 1                           // Raz�o entre freq de amostragem e de chaveamento
#define TSAMPLE                 1.0/(Nsample*PWM_FREQ)      //Per�odo de amorstrage

#define Nb_int                  3                           // N�mero de bra�os do conversor cc/cc (Interleaved)
#define Nbat_series             16                           // N�mero de baterias em s�rie
#define Nbat_string             1                           // N�mero de strings de bateria

// Tamanho dos Buffers de plot
#define RESULTS_BUFFER_SIZE  8*(Nsample*PWM_FREQ/60)        //Ciclos*(Freq. de amostragem)/(freq.fundamental)

// Tamanho do Buffer de aquisi��o de dados e limite de contagem
#define N_data_log  600               // Aquisi��o de 5h
#define COUNT_LIM_LOG  270000         //Aquisi��o a cada 30s

//Constantes
#define DOISPI               6.283185307179586  // 2*PI
#define Ts_div2              TSAMPLE*0.500000

// Ganhos do controlador PI de corrente do conv. dc/dc no modo boost (descarga das baterias)
#define PI_DIS_KP               0.0050265
#define PI_DIS_KI               0.8339
#define PI_DIS_OUTMAX           1
#define PI_DIS_OUTMIN           0

// Ganhos do controlador PI de corrente do conv. dc/dc no modo buck (carga das baterias)
#define PI_ICH_KP               0.0050265
#define PI_ICH_KI               0.8339
#define PI_ICH_OUTMAX           1
#define PI_ICH_OUTMIN           0

// Ganhos do controlador PI de tens�o do conv. dc/dc no modo buck (carga das baterias)
#define PI_VCH_KP               0.7336
#define PI_VCH_KI               0.0922

// Refer�ncias m�ximas de corrente de descarga e carga
#define Ir_dis                   16
#define Ir_ch                    5

// Limites para prote��es
#define OVER_CURRENT_DC_LIMIT_DISCHARGE      25
#define OVER_CURRENT_DC_LIMIT_CHARGE         15
#define DC_OVERVOLTAGE_LIMIT                 650
#define BAT_OVERVOLTAGE_LIMIT                15.5*Nbat_series
#define BAT_UNDERVOLTAGE_LIMIT               9*Nbat_series

// Defini��o das faltas
#define FAULT_OK                 0
#define FAULT_DC_OVERVOLTAGE     1
#define FAULT_DC_UNDERVOLTAGE    2
#define FAULT_OVERCURRENT        3
#define FAULT_THERMAL            4
#define FAULT_VBAT_OVERVOLTAGE   5
#define FAULT_VBAT_UNDERVOLTAGE  6


#endif

