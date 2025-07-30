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
#define CURVA_EM_DUVIDA 3
#define ANGLE_CURVE 90

#define DETECCAO_POR_QUADRADO 1
#define SAIDA_ESQUERDA 0
#define SAIDA_DIREITA 1

#define DELAY_LOST_LINE 500
#define TIME_WITHOUT_LINE 3000 
//  Pinos dos Sensores 
extern const int sensor1_A1;
extern const int sensor2_A2;
extern const int sensor3_A3;
extern const int sensor4_A4;
extern const int sensor5_A5;
extern const int sensor0_curva_A0;
extern const int sensor6_curva_A6;

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

//  Variáveis do Giroscópio 
extern MPU6050 mpu;
extern float gyro_bias_z;

extern int estado_desafio_re; 
extern int lado_primeira_marca_re; 

#endif // CONSTANTS_H