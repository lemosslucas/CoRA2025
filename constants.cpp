#include "constants.h"

// Pinos dos Sensores 
const int sensor1_A1 = 14;
const int sensor2_A2 = 15;
const int sensor3_A3 = 17;
const int sensor4_A4 = 18;
const int sensor5_A5 = 19;
const int sensor0_curva_A0 = 16;
const int sensor6_curva_A6 = 20;

// Pinos dos LEDs
const int LED_LEFT = 11;
const int LED_RIGHT = 12;

// Variáveis de Estado dos Sensores
int SENSOR[5];
int SENSOR_CURVA[2];

// Configurações de Velocidade e Motores
const int velocidadeBaseDireita = 180;
const int velocidadeBaseEsquerda = 180;
int velocidadeDireita = 0;
int velocidadeEsquerda = 0;

// Variáveis do PID
// Constantes para o cálculo do PID
const float Kp = 150, Ki = 0, Kd = 0;

// Constantes para a utilização do método Ultimate Gain (Ziegler-Nichols)
const float Kcr = 150, Pcr = 0.05;
// const float Kp = (0.6 * Kcr), Ki = ((2 * Kp) / Pcr), Kd = ((Kp * Pcr) / 8);

float erro = 0;
float erroAnterior = 0;
float I = 0, P = 0, D = 0, PID = 0;

// Variáveis de Estado dos Desafios
int saida_rotatoria = -1;
bool faixa_de_pedestre = false;
int saidaDesejada = 0;

// Variáveis do Giroscópio
float gyro_bias_z = 0.0;

int estado_desafio_re = 0; // 0 = Normal, 1 = Viu a primeira marca, 2 = Viu a segunda marca
int lado_primeira_marca_re = -1; // 0 = Esquerda, 1 = Direita