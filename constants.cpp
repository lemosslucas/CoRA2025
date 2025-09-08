#include "constants.h"

// Pinos dos Sensores de Linha
const int sensor_curva_esquerda = 2;
const int sensor_esquerda = 4;
const int sensor_esquerda_central = 8;
const int sensor_central = 17; //A3
const int sensor_direita_central = 14; //A0
const int sensor_direita = 15; //A1
const int sensor_curva_direita = 16; //A2

// Variáveis de Estado dos Sensores
int SENSOR[5];
int SENSOR_CURVA[2];

// Giroscópio
const int chipSelect = 10;
float gyro_bias_z = 0.0;

// --- Motores ---
const int OFFSET_MOTORS = 5;
const int velocidadeBase = 210;
const int velocidadeBaseDireita = velocidadeBase - OFFSET_MOTORS;
const int velocidadeBaseEsquerda = velocidadeBase;
int velocidadeDireita = 0;
int velocidadeEsquerda = 0;


// --- Timeouts ---
const int TIMEOUT_90_CURVE = 700;
const int TIMEOUT_FAIXA_PEDESTRE = 5000;
const int TIMEOUT_MARCACAO = 1000;
const int TEMPO_MAX_LED_LIGADO = 1500; // 3 segundos
const int TIME_WITHOUT_LINE = 200;

// --- Tolerâncias ---
const int TOLERANCIA_CURVA_90 = 2;
const int TOLERANCIA_INVERSAO = 15;
const int LIMITE_TOLERANCIA_LINHA_PERDIDA = 50;

// --- PID ---
// Constantes para o cálculo do PID
const float Kp = 135, Ki = 0, Kd = 0;

float erro = 0;
float erroAnterior = 0;
float I = 0, P = 0, D = 0, PID = 0;


// --- LEDs ---
//const int LED_LEFT  = (!debugSD) ? 7 : NULL;
//const int LED_RIGHT = (!debugSD) ? 8 : NULL;
const int LED_RIGHT = NULL;
const int LED_LEFT  = NULL;
int tempoLedLigou = 0;
bool ledLigado = false;


// --- Desafios ---
int saida_rotatoria = -1;
bool faixa_de_pedestre = false;
int saidaDesejada = 0;
int estado_desafio_re = 0; // 0 = Normal, 1 = Viu a primeira marca, 2 = Viu a segunda marca
int lado_primeira_marca_re = -1; // 0 = Esquerda, 1 = Direita
bool inversaoAtiva = false;
int marcacoesDireita = 0, marcacoesEsquerda = 0;
bool jaContouEsquerda = false, jaContouDireita = false;


// --- Debug ---
bool debugMode = false;
bool debugMotor = false;
bool debugSD = true;