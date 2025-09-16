#ifndef CORA2025_H
#define CORA2025_H

#include <Arduino.h>
#include <PID_v1_bc.h>
// Protótipos de funções do CoRA2025.ino que precisam ser acessíveis
// por outros arquivos .cpp do projeto.

void calcula_erro();
void ajusta_movimento();
void ler_sensores();
void imprime_serial();
int contaMarcacao(int estadoSensor, int contagemAtual, bool &jaContou);
void write_sd(int challenge_marker = 0);

extern double Setpoint, PID_Input, PID_Output;
extern PID myPID;

#endif // CORA2025_H