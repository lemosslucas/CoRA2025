#include "challenges.h"
#include "motors.h"
#include "constants.h"
#include "CoRA2025.h"

int calcula_sensores_ativos(int SENSOR[]) {
  int sensoresAtivos = 0;
  for(int i = 0; i < 5; i++) {
    sensoresAtivos += SENSOR[i];
  }
  return sensoresAtivos;
}



int calcula_posicao(int SENSOR[]) {
  int pesos[5] = {-2, -1, 0, 1, 2};
  int somaPesos = 0, somaAtivos = 0;

  for (int i = 0; i < 5; i++) {
    if (SENSOR[i] == PRETO) {
      somaPesos += pesos[i];
      somaAtivos++;
    }
  }

  if (somaAtivos == 0) return 0;
  return somaPesos / somaAtivos;
}



int verifica_curva_90(int SENSOR[], int SENSOR_CURVA[]) {
  int sensores_pretos = calcula_sensores_ativos(SENSOR);

  if (SENSOR_CURVA[0] == PRETO && sensores_pretos >= 2 && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_ESQUERDA;
  }
  if (SENSOR_CURVA[0] == BRANCO && sensores_pretos >= 2 && SENSOR_CURVA[1] == PRETO) {
    return CURVA_DIREITA;
  }
  if (SENSOR[1] == PRETO && SENSOR[3] == PRETO && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_EM_DUVIDA;
  }
  return CURVA_NAO_ENCONTRADA;
}



void turn_90(int curvaEncontrada) {
  unsigned long startTime = millis();
  while ((millis() - startTime < TIMEOUT_90_CURVE)) {
    run(velocidadeBaseDireita, velocidadeBaseEsquerda);
    ler_sensores();
  }
  
  int posicao = calcula_posicao(SENSOR);
  int ajuste = posicao * 10;  
  int anguloFinal = ANGLE_CURVE + ajuste;

  stop_motors();
  delay(200);

  erro = erroAnterior;
  if (curvaEncontrada == CURVA_ESQUERDA) {
    if (debugSD) write_sd(1);
    digitalWrite(LED_LEFT, HIGH);
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(anguloFinal); 
    digitalWrite(LED_LEFT, LOW);
  } else if (curvaEncontrada == CURVA_DIREITA) {
    if (debugSD) write_sd(4);
    digitalWrite(LED_RIGHT, HIGH);
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(anguloFinal);
    digitalWrite(LED_RIGHT, LOW);
  }

  stop_motors();
  delay(100); 
}

float calibrate_gyro(int samples = 200) {
    if (debugMode) Serial.println("Calibrating gyroscope... Keep the robot stationary.");
    
    float sum_gz = 0;
    for (int i = 0; i < samples; i++) {
        mpu.update(); 
        sum_gz += mpu.getGyroZ();
        delay(10); 
    }
            
    float bias_gz = sum_gz / samples;
    
    if (debugMode) {
      Serial.print("Calibration complete. Gyroscope Bias (Gz) = ");
      Serial.println(bias_gz, 4);
    }

    return bias_gz;
}

void turn_until_angle(int target_angle = 90) {
  unsigned long previous_time = millis();
  float angle_z = 0;

  while (abs(angle_z) < target_angle) {
    mpu.update(); 
    float angular_velocity_z = mpu.getGyroZ() - gyro_bias_z;

    unsigned long current_time = millis();
    float delta_time = (current_time - previous_time) / 1000.0; 
    previous_time = current_time;

    angle_z += angular_velocity_z * delta_time;

    if (debugMode) Serial.println(angular_velocity_z);
    delay(10);
  }
  stop_motors();

  if (debugMode) Serial.println("Rotation finished");
}

void turn_to_absolute(float target_angle) {
  mpu.update();
  float angAtual = mpu.getAngleZ();

  float erro = target_angle - angAtual;
  if (erro > 180) erro -= 360;
  if (erro < -180) erro += 360;

  int sentido = (erro > 0) ? 1 : -1;

  unsigned long previous_time = millis();
  float angle_z = 0;

  while (fabs(angle_z) < fabs(erro)) {
    mpu.update(); 
    float angular_velocity_z = mpu.getGyroZ() - gyro_bias_z;

    unsigned long current_time = millis();
    float delta_time = (current_time - previous_time) / 1000.0; 
    previous_time = current_time;

    angle_z += angular_velocity_z * delta_time * sentido;

    if (debugMode) Serial.println(angle_z);
    delay(5);
  }

  stop_motors();
  if (debugMode) Serial.println("Rotation finished");
}

int inverte_sensor(int sensor){
  if (sensor == 1){ 
    return 0;
  } 
  return 1;
}

unsigned long tempoInversaoAtivada = 0;

void definirSetPointInicial() {
  if (debugMode) Serial.println("Definindo setpoint de ângulo inicial...");

  float anguloInicialMedio = 0;
  int amostras = 50;

  for (int i = 0; i < amostras; i++) {
    mpu.update();
    anguloInicialMedio += mpu.getAngleZ();
    delay(5);
  }
  anguloInicialMedio /= amostras;

  if (debugMode) {
    Serial.print("Angulo inicial medido: ");
    Serial.println(anguloInicialMedio);
  }

  int setpoints[] = {0, 90, 180, 270, -90, -180, -270};
  float menorDiferenca = 360.0;
  float setpointEscolhido = 0.0;

  for (int sp : setpoints) {
    float diferenca = abs(anguloInicialMedio - sp);
    if (diferenca < menorDiferenca) {
      menorDiferenca = diferenca;
      setpointEscolhido = sp;
    }
  }

  anguloSetPointGlobal = setpointEscolhido;

  if (debugMode) {
    Serial.print("Setpoint global definido para: ");
    Serial.println(anguloSetPointGlobal);
  }
}



bool linha_recuperada() {
  ler_sensores();
  int posicao = calcula_posicao(SENSOR);
  if (posicao != 0) {
    return true;
  }
  return false;
}



bool verifica_inversao(int SENSOR[], int SENSOR_CURVA[]) {
  int ativos = calcula_sensores_ativos(SENSOR);

  if (ativos == 1 && !inversaoAtiva && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == BRANCO) {
    inversaoAtiva = true;
    tempoInversaoAtivada = millis();
    if (debugSD) write_sd(10);
  }

  if (inversaoAtiva && millis() - tempoInversaoAtivada > 1000) {
    if (ativos >= 3 && SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO) {
        inversaoAtiva = false;
        if (debugSD) write_sd(11);
        inversao_finalizada = true;
        return false;
    }
  }

  if (inversaoAtiva) {
    for (int i = 0; i < 5; i++) {
      SENSOR[i] = inverte_sensor(SENSOR[i]);
    }    
    return true; 
  }

  return false;
}



float anguloMaisProximo(float angAtual) {
  int setpoints[4] = {0, 90, 180, -90};
  float menorDiff = 9999;
  float escolhido = 0;

  for (int i = 0; i < 4; i++) {
    float diff = fabs(angAtual - setpoints[i]);
    if (diff > 180) diff = 360 - diff;
    if (diff < menorDiff) {
      menorDiff = diff;
      escolhido = setpoints[i];
    }
  }
  return escolhido;
}

const int offset_alinhamento = 90;



void alinharParaCardealMaisProximo() {
  mpu.update();
  float angAtual = mpu.getAngleZ() - gyro_bias_z;
  float alvo = anguloMaisProximo(angAtual);

  for (int tentativa = 0; tentativa < 1; tentativa++) {
    mpu.update();
    angAtual = mpu.getAngleZ();

    float erro = alvo - angAtual;
    if (erro > 180) erro -= 360;
    if (erro < -180) erro += 360;

    if (abs(erro) < 3) {
      stop_motors();
      if (debugMode) {
        Serial.print("Alinhado em: ");
        Serial.println(angAtual);
      }
      return;
    }

    if (erro < 0) {
      turn_right(velocidadeBaseDireita - offset_alinhamento, velocidadeBaseEsquerda - offset_alinhamento);
      turn_to_absolute(alvo);
    } else {
      turn_left(velocidadeBaseDireita - offset_alinhamento, velocidadeBaseEsquerda - offset_alinhamento);
      turn_to_absolute(alvo);
    }

    stop_motors();
    delay(100);
  }

  mpu.update();
  angAtual = mpu.getAngleZ();
  if (debugMode) {
    Serial.print("Angulo final apos alinhamento: ");
    Serial.println(angAtual);
  }
}



void realiza_faixa_de_pedestre() {
  run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
  delay(150);
  stop_motors();
  delay(TIMEOUT_FAIXA_PEDESTRE);

  if (debugSD) write_sd(2);
  alinharParaCardealMaisProximo();

  if (debugMode) Serial.println("Alinhamento concluído. Atravessando a faixa...");
  float anguloReferencia = mpu.getAngleZ();
  float Kp_gyro = 50.0;

  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT_PERIODO_FAIXA) {
    mpu.update();
    float anguloAtual = mpu.getAngleZ();
    float erro = anguloReferencia - anguloAtual;
    int correcao = (int)(Kp_gyro * erro);

    int velD = constrain(velocidadeBaseDireita - correcao, 0, 255);
    int velE = constrain(velocidadeBaseEsquerda + correcao, 0, 255);

    run(velD, velE);
    ler_sensores();
  }
  stop_motors();
  delay(100);
} 



int determina_saida_curva(int marcacoesEsquerda, int marcacoesDireita) {
  if (marcacoesDireita < marcacoesEsquerda) {
    return SAIDA_DIREITA;
  } else {
    return SAIDA_ESQUERDA;
  }
  // e o curva em duvida???
}



int determina_saida_rotatoria(int saidaCurva, int numeroDeMarcas) {
  if (numeroDeMarcas == 2) {
    saidaDesejada = 1;
  } else if (numeroDeMarcas == 3) {
    saidaDesejada = 2;
  } else if (numeroDeMarcas >= 4) { 
    saidaDesejada = 3;
  }
  return saidaDesejada;
}

void realiza_rotatoria(int saidaCurva, int saidaDesejada) {
  if (debugSD) write_sd(8);
  
  bool aguardando_realinhar = false;
  static int contador_deteccao_saida = 0;
  const int TOLERANCIA_SAIDA = 2;
  int saidaAtual = 0;

  if (saidaCurva == SAIDA_ESQUERDA) {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  } else if (saidaCurva == SAIDA_DIREITA) {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  }

  while (saidaAtual < saidaDesejada) {
    calcula_erro();
    ajusta_movimento();

    bool saida_detectada = (SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == BRANCO);
    bool robo_realinhado = (SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO);

    if (!aguardando_realinhar) {
      if (saida_detectada) {
        contador_deteccao_saida++;
      } else {
        contador_deteccao_saida = 0;
      }
      if (contador_deteccao_saida >= TOLERANCIA_SAIDA) {
        saidaAtual++;
        aguardando_realinhar = true;
        contador_deteccao_saida = 0;
      }
    } else {
      if (robo_realinhado) {
        aguardando_realinhar = false;
      }
    }
  }

  if (saidaCurva == SAIDA_ESQUERDA) {
      turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
      turn_until_angle(90);
  } else {
      turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
      turn_until_angle(90);
  }
}



void verifica_estado_led() {
  if (ledLigado) {
    if (millis() - tempoLedLigou >= TEMPO_MAX_LED_LIGADO) {
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, LOW);
      ledLigado = false;
    }
  }
}



bool tenta_recuperar_linha() {
  unsigned long tempoPerdido = millis();
  int leiturasValidas = 0;

  if (debugSD) write_sd(5);

  while (millis() - tempoPerdido < TIME_WITHOUT_LINE) {
    run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
    delay(50);
    ler_sensores();

    if (SENSOR[1] == PRETO || SENSOR[2] == PRETO || SENSOR[3] == PRETO) {
      leiturasValidas++;
      if (leiturasValidas >= 3) {
        stop_motors();
        return true;
      }
    } else {
      leiturasValidas = 0;
    }
  }
  return false;
}



int conta_marcacao(int estadoSensor, int contagemAtual, bool &jaContou) {
  if (estadoSensor == PRETO && !jaContou) {
    jaContou = true; 
    return contagemAtual + 1;
  } else if (estadoSensor == BRANCO) {
    jaContou = false; 
  }
  return contagemAtual; 
}



void analisa_marcacoes() {
  // Zera as variáveis globais para uma nova análise limpa
  marcacoesEsquerda = 0;
  marcacoesDireita = 0;
  ultimo_lado_re = -1; 
  jaContouEsquerda = false; 
  jaContouDireita = false;

  unsigned long tempoInicioAnalise = millis();

  while (millis() - tempoInicioAnalise < TIMEOUT_MARCACAO) {
    // Guarda o último lado detectado
    int contagemAnteriorE = marcacoesEsquerda;
    marcacoesEsquerda = conta_marcacao(SENSOR_CURVA[0], marcacoesEsquerda, jaContouEsquerda); 
    if (marcacoesEsquerda > contagemAnteriorE) {
        ultimo_lado_re = 0; // 0 para Esquerda
    }
    
    int contagemAnteriorD = marcacoesDireita;
    marcacoesDireita = conta_marcacao(SENSOR_CURVA[1], marcacoesDireita, jaContouDireita);
    if (marcacoesDireita > contagemAnteriorD) {
        ultimo_lado_re = 1; // 1 para Direita
    }

    // Segue a linha enquanto analisa para não se perder
    calcula_erro();
    calcula_PID();
    ajusta_movimento();
    if (debugSD) write_sd(9);
  }
  
  stop_motors();
  delay(50);
}

// +++ ADICIONADA A NOVA FUNÇÃO 'realiza_marcha_re' +++
void realiza_marcha_re(int lado_da_curva) {
  if (debugSD) write_sd(6); // Use um código de log para a Ré

  // 1. Dá ré por uma distância definida (ajuste este delay!)
  run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
  delay(400); // <-- VALOR CRÍTICO: Ajuste conforme necessário.

  stop_motors();
  delay(200);

  // 2. Vira para o lado do ÚLTIMO quadrado visto
  if (lado_da_curva == 1) { // 1 = Direita
    turn_90(CURVA_DIREITA);
  } else { // 0 = Esquerda
    turn_90(CURVA_ESQUERDA);
  }
  
  // Avança um pouco para garantir que encontrou a linha
  run(velocidadeBaseDireita, velocidadeBaseEsquerda);
  delay(200);
}

void area_de_parada() {
  stop_motors();
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  tempoLedLigou = millis();
  ledLigado = true;
  if (debugMode) Serial.println("Área de parada detectada. Robô parado.");
  if (debugSD) write_sd(3);
  while(true);
}

void test_motors() {
  run(200, 200);
  delay(2000);
  stop_motors();
  delay(1000);

  run(200, 200);
  delay(2000);
  stop_motors();
  delay(1000);
}