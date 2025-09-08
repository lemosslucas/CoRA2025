#ifndef CORA2025_H
#define CORA2025_H

#include <Arduino.h>

// Protótipos de funções do CoRA2025.ino que precisam ser acessíveis
// por outros arquivos .cpp do projeto.

void calcula_erro();
void ajusta_movimento();
void ler_sensores();
void imprime_serial();
void calcula_PID();
int contaMarcacao(int estadoSensor, int contagemAtual, bool &jaContou);
void write_sd(int challenge_marker = 0);
#endif // CORA2025_H