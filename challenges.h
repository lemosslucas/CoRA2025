#ifndef CHALLENGES_H
#define CHALLENGES_H

#include <Arduino.h>

int verifica_curva_90(int SENSOR[], int SENSOR_CURVA[]);
void turn_90(int curvaEncontrada);
int inverte_sensor(int SENSOR);
bool verifica_inversao(int SENSOR[], int SENSOR_CURVA[]);
int calcula_sensores_ativos(int SENSOR[]);
void realiza_faixa_de_pedestre();
float calibrate_gyro(int samples = 200);
void realiza_rotatoria(int saidaCurva, int saidaDesejada);
void turn_until_angle(int target_angle);
void realiza_marcha_re(int lado_da_curva);
int determina_saida_curva(int marcacoesEsquerda, int marcacoesDireita);
int determina_saida_rotatoria(int saidaCurva, int numeroDeMarcas);
void test_motors();
void verifica_estado_led();
bool tenta_recuperar_linha();
void analisa_marcacoes();
void area_de_parada();
int calcula_posicao(int SENSOR[]);

void definirSetPointInicial_Quadrante();
void girarParaAnguloAbsoluto(float anguloAlvo, int velocidadeGiro = 150);
void alinharParaCardealMaisProximo();




#endif
