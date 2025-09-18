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
const int velocidadeBase = (!arrancadaMode) ? 210 : 255;
const int velocidadeBaseDireita = velocidadeBase - OFFSET_MOTORS;
const int velocidadeBaseEsquerda = velocidadeBase;
int velocidadeDireita = 0;
int velocidadeEsquerda = 0;


// --- Timeouts ---
const int TIMEOUT_FAIXA_PEDESTRE = 5100;
const int TIMEOUT_MARCACAO = 400; // 550
const int TEMPO_MAX_LED_LIGADO = 1500; // 3 segundos
const int TIME_WITHOUT_LINE = 200;
const int TIMEOUT_PERIODO_FAIXA = 2000;

// --- Tolerâncias ---
const int LIMITE_TOLERANCIA_LINHA_PERDIDA = 50;

// --- PID ---
// Constantes para o cálculo do PID
const float Kp = (!arrancadaMode) ? 90 : 150; 
const float Ki = (!arrancadaMode) ? 0 : 0;
const float Kd = (!arrancadaMode) ? 5 : 0;

float erro = 0;
float erroAnterior = 0;
float I = 0, P = 0, D = 0, PID = 0;

// --- LEDs ---
const int LEDS = 7;
int tempoLedLigou = 0;
bool ledLigado = false;


// --- Desafios ---
int saida_rotatoria = -1;
bool faixa_de_pedestre = false;
int saidaDesejada = 0;
bool inversaoAtiva = false;
int marcacoesDireita = 0, marcacoesEsquerda = 0;
bool jaContouEsquerda = false, jaContouDireita = false;
bool inversao_finalizada = false;
int ultima_posicao_linha = 0;

const unsigned long DEBOUNCE_TEMPO_CURVA = 1000;
unsigned long tempoUltimaCurva = 0;

unsigned long tempoMarcacaoDireita = 0;
unsigned long tempoMarcacaoEsquerda = 0;
const int TOLERANCIA_TEMPO_SIMULTANEO = 300;
// --- Debug ---
bool debugMode = false;
bool debugMotor = false;
bool debugSD = false;
bool arrancadaMode = false;