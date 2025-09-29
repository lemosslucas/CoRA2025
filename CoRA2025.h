#ifndef CORA2025_H
#define CORA2025_H

#include <Arduino.h>

// Protótipos de funções do CoRA2025.ino que precisam ser acessíveis
// por outros arquivos .cpp do projeto.

void calculate_error();
void adjust_movement();
void read_sensors();
void print_serial();
void calculate_PID();
int count_marker(int sensorState, int currentCount, bool &alreadyCounted);
void write_sd(int challenge_marker = 0);
#endif // CORA2025_H