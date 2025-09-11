#include "challenges.h"
#include "motors.h"
#include "constants.h"
#include "CoRA2025.h"

/**
 * @brief Calculates the number of active sensors.
 * 
 * An active sensor is one that detects the color black (value 1).
 * 
 * @param SENSOR Array with the state of the 5 main line-following sensors.
 * @return int The total number of active sensors.
 */
int calcula_sensores_ativos(int SENSOR[]) {
  int sensoresAtivos = 0;
  // Calculates the number of active sensors
  for(int i = 0; i < 5; i++) {
    sensoresAtivos += SENSOR[i];
  }
  
  // Returns the number of active sensors
  return sensoresAtivos;
}

/**
 * @brief Determines if the robot should perform a 90-degree turn.
 * 
 * Esta função analisa o estado dos sensores principais e de curva para identificar
 * um padrão de curva de 90 graus. Para evitar falsos positivos devido a ruídos
 * nos sensores, ela exige que uma curva seja detectada por um número mínimo de
 * leituras consecutivas (3, por padrão) antes de confirmar a curva e retornar sua direção.
 * 
 * @param SENSOR Array containing the states of the main sensors.
 * @param SENSOR_CURVA Array containing the states of the curve detection sensors.
 * 
 * @return int An integer representing the turn direction:
 * - `CURVA_ESQUERDA` se uma curva à esquerda for confirmada.
 * - `CURVA_DIREITA` se uma curva à direita for confirmada.
 * - `CURVA_EM_DUVIDA` se a direção da curva for confirmada, mas incerta.
 * - `CURVA_NAO_ENCONTRADA` se nenhuma curva for detectada ou o limiar de detecção não for atingido.
 */

int verifica_curva_90(int SENSOR[], int SENSOR_CURVA[]) {
  int sensores_pretos = calcula_sensores_ativos(SENSOR);

  // Curva à direita
  if (SENSOR_CURVA[0] == PRETO && sensores_pretos >= 2 && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_ESQUERDA;
  }

  // Curva à esquerda
  if (SENSOR_CURVA[0] == BRANCO && sensores_pretos >= 2 && SENSOR_CURVA[1] == PRETO) {
    return CURVA_DIREITA;
  }

  // Caso de dúvida (linha reta com marca estranha)
  if (SENSOR[1] == PRETO && SENSOR[3] == PRETO && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_EM_DUVIDA;
  }

  return CURVA_NAO_ENCONTRADA;
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

  if (somaAtivos == 0) return 0; // Falha de leitura
  return somaPesos / somaAtivos; // valor médio
}


/**
 * @brief Executes a 90-degree turn using the gyroscope.
 * 
 * Based on the detected turn direction, this function stops the robot for
 * stability and then performs a precise 90-degree turn.
 * 
 * @param curvaEncontrada The direction of the turn, e.g., `CURVA_ESQUERDA` or `CURVA_DIREITA`.
 */
void turn_90(int curvaEncontrada) {

  unsigned long startTime = millis();
  // This timeout prevents the robot from getting stuck if it misreads the sensors.
  while ((millis() - startTime < TIMEOUT_90_CURVE)) {
    // Move straight forward, not using line-following logic here.
    run(velocidadeBaseDireita, velocidadeBaseEsquerda);
    ler_sensores(); // Keep updating sensor values to check the condition.
  }
  
  int posicao = calcula_posicao(SENSOR);

  // Ajuste proporcional (ex.: 10 graus por posição)
  int ajuste = posicao * 10;  

  int anguloFinal = ANGLE_CURVE + ajuste;

  // Stop the car for greater stability
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


/**
 * @brief Measures the gyroscope's Z-axis bias while the robot is stationary.
 * 
 * This function should be called during setup to calibrate the gyroscope.
 * It takes multiple readings and calculates the average drift (bias).
 * 
 * @param samples The number of samples to collect for calibration.
 * @return float The calculated bias value for the Z-axis.
 */
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

/**
 * @brief Rotates the robot until a target angle is reached.
 * 
 * This function uses the gyroscope to perform a precise rotation. It continuously
 * integrates the angular velocity to track the current angle and stops the motors
 * when the target angle is achieved. It relies on a pre-calibrated `gyro_bias_z`.
 * 
 * @param target_angle The desired angle of rotation in degrees.
 */
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

  // calcula erro normalizado
  float erro = target_angle - angAtual;
  if (erro > 180) erro -= 360;
  if (erro < -180) erro += 360;

  // decide direção
  int sentido = (erro > 0) ? 1 : -1;

  unsigned long previous_time = millis();
  float angle_z = 0;

  // roda até percorrer o erro relativo
  while (fabs(angle_z) < fabs(erro)) {
    mpu.update(); 
    float angular_velocity_z = mpu.getGyroZ() - gyro_bias_z;

    unsigned long current_time = millis();
    float delta_time = (current_time - previous_time) / 1000.0; 
    previous_time = current_time;

    angle_z += angular_velocity_z * delta_time * sentido; // aplica direção

    if (debugMode) Serial.println(angle_z);
    delay(5);
  }

  stop_motors();
  if (debugMode) Serial.println("Rotation finished");
}

/**
 * @brief Inverts the value obtained from a sensor.
 * 
 * This function ensures the robot can follow an inverted line 
 * (e.g., white line on a black surface) by applying NOT logic.
 * 
 * @param sensor The sensor output value (0 for WHITE, 1 for BLACK).
 * @return int The inverted sensor value (1 becomes 0, 0 becomes 1).
 */
int inverte_sensor(int sensor){
  if (sensor == 1){ 
    return 0;
  } 
  return 1;
}

unsigned long tempoInversaoAtivada = 0;

/**
 * @brief Checks for a track color inversion.
 * 
 * This function determines if a track color inversion (black/white to white/black)
 * has occurred. If detected, it inverts the main sensor values to allow the
 * robot to continue following the line.
 * 
 * @param SENSOR Array containing the states of the main sensors. This array is modified in place.
 * @param SENSOR_CURVA Array containing the states of the curve sensors.
 * @note The current implementation detects an inversion if exactly one main sensor is active.
 * @note The SENSOR_CURVA parameter is not currently used in this function.
 * 
 * @return bool Returns true if an inversion was detected and handled, false otherwise.
 */
// Coloque esta versão corrigida no seu arquivo challenges.cpp
/**
 
@brief Calcula o ângulo inicial do robô e define o setpoint global.
Esta função deve ser chamada no setup(), após a calibração do giroscópio.
Ela mede a orientação atual do robô, encontra qual dos ângulos cardeais
(0, 90, 180, 270) está mais próximo e armazena esse valor na variável
global anguloSetPointGlobal. Isso alinha o sistema de referência do robô
com a pista.*/
void definirSetPointInicial() {
  if (debugMode) Serial.println("Definindo setpoint de ângulo inicial...");

  float anguloInicialMedio = 0;
  int amostras = 50; // Pega 50 amostras para ter uma média estável

  // Tira uma média das leituras iniciais para reduzir ruído
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

  // Array com os ângulos de referência possíveis
  int setpoints[] = {0, 90, 180, 270, -90, -180, -270};
  float menorDiferenca = 360.0;
  float setpointEscolhido = 0.0;

  // Itera para encontrar o setpoint mais próximo
  for (int sp : setpoints) {
    float diferenca = abs(anguloInicialMedio - sp);
    if (diferenca < menorDiferenca) {
      menorDiferenca = diferenca;
      setpointEscolhido = sp;
    }
  }

  // Atribui o valor encontrado à variável global
  anguloSetPointGlobal = setpointEscolhido;

  if (debugMode) {
    Serial.print("Setpoint global definido para: ");
    Serial.println(anguloSetPointGlobal);
  }
}
bool linha_recuperada() {
  ler_sensores();
  int posicao = calcula_posicao(SENSOR);

  // Se encontrou uma posição válida (linha voltou a ser visível)
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

  if (inversaoAtiva && millis() - tempoInversaoAtivada > 1000) { // só desliga depois de 200ms
    if (ativos >= 3 && SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO) {
        inversaoAtiva = false;
        if (debugSD) write_sd(11);
        inversao_finalizada = true;
        return false;
    }
  }


  // Se o modo de inversão está ativo, inverta a leitura dos sensores.
  if (inversaoAtiva) {
    for (int i = 0; i < 5; i++) {
      SENSOR[i] = inverte_sensor(SENSOR[i]);
    }    
    return true; 
  }

  // Se não entrou na lógica, retorna false.
  return false;
}

/**
 * @brief Handles the pedestrian crossing challenge.
 * 
 * This function simulates the robot's behavior at a pedestrian crossing.
 * It waits for a minimum required time (6 seconds) and then moves forward
 * to cross the track.
 */
 /**
 
@brief Alinha o robô com o setpoint global, escolhendo o caminho mais curto.*
Calcula a diferença entre o ângulo atual e o setpoint global, normaliza
essa diferença para o intervalo [-180, 180] para encontrar o caminho
mais curto, e então executa a rotação para a esquerda ou direita conforme
necessário.*/
float anguloMaisProximo(float angAtual) {
  int setpoints[4] = {0, 90, 180, -90}; // -90 equivale a 270
  float menorDiff = 9999;
  float escolhido = 0;

  for (int i = 0; i < 4; i++) {
    float diff = fabs(angAtual - setpoints[i]);
    if (diff > 180) diff = 360 - diff; // correção wrap-around
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

  // tenta alinhar até estar dentro da tolerância
  for (int tentativa = 0; tentativa < 1; tentativa++) { // até 3 ajustes
    mpu.update();
    angAtual = mpu.getAngleZ();

    float erro = alvo - angAtual;
    if (erro > 180) erro -= 360;
    if (erro < -180) erro += 360;

    if (abs(erro) < 3) { // tolerância de 3 graus
      stop_motors();
      if (debugMode) {
        Serial.print("Alinhado em: ");
        Serial.println(angAtual);
      }
      return; // já está alinhado
    }

    if (erro < 0) {
      // gira um pouco à direita
      turn_right(velocidadeBaseDireita - offset_alinhamento, velocidadeBaseEsquerda - offset_alinhamento);
      turn_to_absolute(alvo);
    } else {
      // gira um pouco à esquerda
      turn_left(velocidadeBaseDireita - offset_alinhamento, velocidadeBaseEsquerda - offset_alinhamento);
      turn_to_absolute(alvo);
    }

    stop_motors();
    delay(100);
  }

  // última leitura para confirmar
  mpu.update();
  angAtual = mpu.getAngleZ();
  if (debugMode) {
    Serial.print("Angulo final apos alinhamento: ");
    Serial.println(angAtual);
  }
}

void realiza_faixa_de_pedestre() {
  // --- Dá ré curta para se posicionar antes da faixa ---
  run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
  delay(150);
  stop_motors();
  delay(TIMEOUT_FAIXA_PEDESTRE); // Pausa para estabilizar o MPU

  if (debugSD) write_sd(2);

  // alinah perfeito
  alinharParaCardealMaisProximo();

  // --- FASE 2: Travessia da faixa em linha reta ---
  if (debugMode) Serial.println("Alinhamento concluído. Atravessando a faixa...");
  float anguloReferencia = mpu.getAngleZ();
  float Kp_gyro = 50.0; // ganho proporcional, ajustar na prática

  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT_PERIODO_FAIXA) {
    mpu.update();
    float anguloAtual = mpu.getAngleZ();
    float erro = anguloReferencia - anguloAtual;

    // Correção proporcional simples
    int correcao = (int)(Kp_gyro * erro);

    int velD = constrain(velocidadeBaseDireita - correcao, 0, 255);
    int velE = constrain(velocidadeBaseEsquerda + correcao, 0, 255);

    run(velD, velE);
    ler_sensores();

    // Se a linha normal for encontrada novamente, encerra
  
  }

  stop_motors();
  delay(100);
} 
/**
 * @brief Determines the exit direction for a challenge based on marker counts.
 * 
 * Compares the number of markers detected on the left and right sides.
 * The robot should exit on the side with fewer markers.
 * 
 * @param marcacoesEsquerda The number of markers detected on the left.
 * @param marcacoesDireita The number of markers detected on the right.
 * @return int The determined exit side (`SAIDA_DIREITA` or `SAIDA_ESQUERDA`).
 */
int determina_saida_curva(int marcacoesEsquerda, int marcacoesDireita) {
  if (marcacoesDireita < marcacoesEsquerda) {
    return SAIDA_DIREITA;
  } else {
    return SAIDA_ESQUERDA;
  }
}


/**
 * @brief Determines the roundabout exit based on the number of detected markers.
 * 
 * Based on the total number of markers seen before entering the roundabout,
 * this function sets the desired exit number.
 * 
 * @param saidaCurva The direction of the curve to enter the roundabout (currently unused).
 * @param numeroDeMarcas The total number of markers detected.
 * @return int The desired exit number (1, 2, or 3).
 */
int determina_saida_rotatoria(int saidaCurva, int numeroDeMarcas) {
  if (numeroDeMarcas == 2) {
    saidaDesejada = 1; // 1ª Saída
  } else if (numeroDeMarcas == 3) {
    saidaDesejada = 2; // 2ª Saída
  } else if (numeroDeMarcas >= 4) { 
    saidaDesejada = 3; // 3ª Saída
  }
  
  return saidaDesejada;
}

/**
 * @brief Navigates the roundabout challenge.
 * 
 * The robot enters the roundabout, follows the line, and counts exits until
 * it reaches the `saidaDesejada`. It then takes that exit.
 * 
 * @param saidaCurva The direction to turn to enter the roundabout.
 * @param saidaDesejada The target exit number to take.
 */
void realiza_rotatoria(int saidaCurva, int saidaDesejada) {
  if (debugSD) write_sd(8);
  
  // Flag para controlar se estamos procurando uma saída ou esperando passar por uma já contada
  bool aguardando_realinhar = false;
  
  // Contador para adicionar tolerância à detecção da saída
  static int contador_deteccao_saida = 0;
  const int TOLERANCIA_SAIDA = 2; // Exige 2 leituras consecutivas para confirmar a saída

  // Zera a contagem no início do desafio
  int saidaAtual = 0;

  // 1. Vira 90 graus para entrar na rotatória (usando o giroscópio)
  if (saidaCurva == SAIDA_ESQUERDA) {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  } else if (saidaCurva == SAIDA_DIREITA) {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  }

  // 2. Navega na rotatória até encontrar a saída correta
  while (saidaAtual < saidaDesejada) {
    // Lógica principal de seguir linha é executada em todos os ciclos
    calcula_erro();
    ajusta_movimento();

    // Assumindo que a rotatória é para a direita (sentido anti-horário)
    bool saida_detectada = (SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == BRANCO);
    
    // Condição que indica que o robô está de volta à curva principal
    bool robo_realinhado = (SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO);

    // Lógica principal de contagem
    if (!aguardando_realinhar) {
      // procura a saida
      if (saida_detectada) {
        contador_deteccao_saida++; // Incrementa contador se vir uma possível saída
      } else {
        contador_deteccao_saida = 0; // Zera se a condição falhar
      }

      // Se a saída foi vista por leituras suficientes, conta e ativa a flag de espera
      if (contador_deteccao_saida >= TOLERANCIA_SAIDA) {
        saidaAtual++;
        aguardando_realinhar = true; // Ativa a flag para parar de procurar
        contador_deteccao_saida = 0; // Zera o contador para a próxima busca
      }
    } else {
      // Se a flag está ativa, esperamos o robô se realinhar para desativá-la
      if (robo_realinhado) {
        aguardando_realinhar = false; // Desativa a flag, permitindo a busca pela próxima saída
      }
    }
  }

  // sai da rotatória, o robô já estará alinhado com a saída, basta virar 90 graus
  if (saidaCurva == SAIDA_ESQUERDA) {
      turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
      turn_until_angle(90);
  } else {
      turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
      turn_until_angle(90);
  }
}

void verifica_estado_led() {
  // verifica se o led ta ligado
  if (ledLigado) {
    // verifica se o led ta ligado por mais de 3 segundos
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

  if (debugSD) write_sd(5); // perda de linha

  while (millis() - tempoPerdido < TIME_WITHOUT_LINE) {
    run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
    delay(50);
    ler_sensores();

    // critério: sensor central detecta linha
    if (SENSOR[1] == PRETO || SENSOR[2] == PRETO || SENSOR[3] == PRETO) {
      leiturasValidas++;
      if (leiturasValidas >= 3) { // exige 3 leituras consecutivas
        stop_motors();
        return true;
      }
    } else {
      leiturasValidas = 0; // perdeu de novo
    }
  }

  //stop_motors();
  return false;
}

/**
 * @brief Counts a marker (black square) using rising edge detection.
 * 
 * Increments the marker count only on the transition from WHITE to BLACK.
 * A control flag (`jaContou`) is used to ensure that the same marker
 * is not counted multiple times while the sensor remains over it.
 * 
 * @param estadoSensor The current state of the sensor (PRETO or BRANCO).
 * @param contagemAtual The current value of the marker counter.
 * @param jaContou Reference to a flag indicating if the current marker has already been counted.
 * @return int The updated marker count.
 */
int conta_marcacao(int estadoSensor, int contagemAtual, bool &jaContou) {
  if (estadoSensor == PRETO && !jaContou) {
    // Activate the lock to prevent recounting
    jaContou = true; 
    return contagemAtual + 1;
  } else if (estadoSensor == BRANCO) {
    // Reset the lock when white is seen
    jaContou = false; 
  }
  // Return the count unchanged
  return contagemAtual; 
}


void analisa_marcacoes() {
  unsigned long tempoUltimaDeteccao = millis();

  // If there is a curve, store the number of markers
  while(erro != LINHA_NAO_DETECTADA && (millis() - tempoUltimaDeteccao < TIMEOUT_MARCACAO)) {
    // Update sensor values
    ler_sensores();

    if (SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO) {
      // Se detectou, reseta o cronômetro do timeout
      tempoUltimaDeteccao = millis();
    }
    
    // conta as marcacoes
    marcacoesEsquerda = conta_marcacao(SENSOR_CURVA[0], marcacoesEsquerda, jaContouEsquerda);
    marcacoesDireita = conta_marcacao(SENSOR_CURVA[1], marcacoesDireita, jaContouDireita);

    // Ensure the robot stays on the line
    calcula_erro();
    calcula_PID();
    ajusta_movimento();
    if (debugSD) write_sd(9);
  }
}

void area_de_parada() {
  stop_motors();
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
  tempoLedLigou = millis();
  ledLigado = true;
  if (debugMode) Serial.println("Área de parada detectada. Robô parado.");
  if (debugSD) write_sd(3); // Log challenge 3: Stop area
  while(true);
}

void test_motors() {
    //run(velocidadeBaseDireita, velocidadeBaseEsquerda);
  //delay(3000);
  //stop_motors();
  //delay(1000);
  //run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
  //delay(3000);
  //stop_motors();
  //delay(1000);
  //turn_90(CURVA_DIREITA);
  //stop_motors();
 // delay(1000);
  //turn_90(CURVA_ESQUERDA);
 // stop_motors();
  //delay(1000);

  run(200, 200);
  delay(2000);
  stop_motors();
  delay(1000);


  run(200, 200);
  delay(2000);
  stop_motors();
  delay(1000);
}