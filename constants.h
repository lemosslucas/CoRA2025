#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <MPU6050_tockn.h>

//  Definições Globais
#define BRANCO 0
#define PRETO 1

//  Configurações do PID e Linha 
#define OFFSET 0
#define LINHA_NAO_DETECTADA -5
#define QUANTIDADE_TOTAL_SENSORES 5

//  Configurações de Desafios 
#define CURVA_NAO_ENCONTRADA 0
#define CURVA_ESQUERDA 1
#define CURVA_DIREITA 2
#define CURVA_EM_DUVIDA CURVA_DIREITA // mudar no dia
#define ANGLE_CURVE 90

#define DETECCAO_POR_QUADRADO 1
#define SAIDA_ESQUERDA 0
#define SAIDA_DIREITA 1

#define DELAY_LOST_LINE 1000
#define TIME_WITHOUT_LINE 3000

//  Pinos dos Sensores 
extern const int sensor_curva_esquerda;
extern const int sensor_esquerda;
extern const int sensor_esquerda_central;
extern const int sensor_central;
extern const int sensor_direita_central;
extern const int sensor_direita;
extern const int sensor_curva_direita;

//  Pinos dos LEDs
extern const int LED_LEFT;
extern const int LED_RIGHT;

//  Variáveis de Estado dos Sensores 
extern int SENSOR[5];
extern int SENSOR_CURVA[2];

//  Configurações de Velocidade e Motores 
extern const int velocidadeBaseDireita;
extern const int velocidadeBaseEsquerda;
extern int velocidadeDireita;
extern int velocidadeEsquerda;

//  Variáveis do PID 
extern const float Kp, Ki, Kd;
extern const float Kcr, Pcr;
extern float erro;
extern float erroAnterior;
extern float I, P, D, PID;

//  Variáveis de Estado dos Desafios 
extern int saida_rotatoria;
extern bool faixa_de_pedestre;
extern int saidaDesejada;
extern bool debugMode;
extern bool debugMotor;
extern bool debugSD;


//  Variáveis do Giroscópio 
extern MPU6050 mpu;
extern float gyro_bias_z;

extern int estado_desafio_re; 
extern int lado_primeira_marca_re; 

extern int tempoLedLigou;
extern const int TEMPO_MAX_LED_LIGADO;
extern bool ledLigado;
#endif // CONSTANTS_H