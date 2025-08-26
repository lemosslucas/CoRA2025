#include "constants.h"

// Pinos dos Sensores 
const int sensor_curva_esquerda = 2;
const int sensor_esquerda = 4;
const int sensor_esquerda_central = 8; 
const int sensor_central = 17; //A3
const int sensor_direita_central = 14; //A0
const int sensor_direita = 15; //A1
const int sensor_curva_direita = 16; //A2

const int chipSelect = 10;

// Pinos dos LEDs
const int LED_LEFT = NULL;
const int LED_RIGHT = NULL;

// Variáveis de Estado dos Sensores
int SENSOR[5];
int SENSOR_CURVA[2];

// Configurações de Velocidade e Motores
const int OFFSET_MOTORS = 5;
const int velocidadeBase = 210;
const int velocidadeBaseDireita = velocidadeBase - OFFSET_MOTORS;
const int velocidadeBaseEsquerda = velocidadeBase;
int velocidadeDireita = 0;
int velocidadeEsquerda = 0;
const int LIMITE_TOLERANCIA_LINHA_PERDIDA = 5;

// Variáveis do PID
// Constantes para o cálculo do PID
const float Kp = 100, Ki = 0.5, Kd = 5;

// Constantes para a utilização do método Ultimate Gain (Ziegler-Nichols)
//const float Kcr = 150, Pcr = 0.05;
// const float Kp = (0.6 * Kcr), Ki = ((2 * Kp) / Pcr), Kd = ((Kp * Pcr) / 8);


float erro = 0;
float erroAnterior = 0;
float I = 0, P = 0, D = 0, PID = 0;

// Variáveis de Estado dos Desafios
int saida_rotatoria = -1;
bool faixa_de_pedestre = false;
int saidaDesejada = 0;

// variaveis para debug
bool debugMode = false;
bool debugMotor = false;
bool debugSD = true;

// Variáveis do Giroscópio
float gyro_bias_z = 0.0;

int estado_desafio_re = 0; // 0 = Normal, 1 = Viu a primeira marca, 2 = Viu a segunda marca
int lado_primeira_marca_re = -1; // 0 = Esquerda, 1 = Direita

int tempoLedLigou = 0;
const int TEMPO_MAX_LED_LIGADO = 1500; // 3 segundos
bool ledLigado = false;

const int TIMEOUT_90_CURVE = 1000;
const int TOLERANCIA_CURVA_90 = 3;
const int TOLERANCIA_INVERSAO = 3;
const int TIMEOUT_FAIXA_PEDESTRE = 5000;
bool inversaoAtiva = false;