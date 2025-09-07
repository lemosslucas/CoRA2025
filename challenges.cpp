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
  // verifica se a curva é para esquerda
  if (SENSOR_CURVA[0] == BRANCO  && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO && SENSOR[4] == PRETO && SENSOR_CURVA[1] == PRETO
  || calcula_sensores_ativos(SENSOR) == 3 && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == PRETO) {
    return CURVA_DIREITA;
  } else if (SENSOR_CURVA[0] == PRETO && SENSOR[0] == PRETO && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO  && SENSOR_CURVA[1] == BRANCO
 || calcula_sensores_ativos(SENSOR) == 3 && SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_ESQUERDA;
  } else if (SENSOR_CURVA[0] == BRANCO && SENSOR[0] == BRANCO && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO && SENSOR[4] == BRANCO && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_EM_DUVIDA;
  }

  return CURVA_NAO_ENCONTRADA;
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
  while (calcula_sensores_ativos(SENSOR) >= 3 && (millis() - startTime < TIMEOUT_90_CURVE)) {
    // Move straight forward, not using line-following logic here.
    run(velocidadeBaseDireita, velocidadeBaseEsquerda);
    ler_sensores(); // Keep updating sensor values to check the condition.
  }
  
  // Stop the car for greater stability
  stop_motors();
  delay(200);

  if (curvaEncontrada == CURVA_ESQUERDA) {
    if (debugSD) write_sd(1);
    digitalWrite(LED_LEFT, HIGH);
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(ANGLE_CURVE);
    digitalWrite(LED_LEFT, LOW);
  } else if (curvaEncontrada == CURVA_DIREITA) {
    if (debugSD) write_sd(4);
    digitalWrite(LED_RIGHT, HIGH);
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(ANGLE_CURVE);
    digitalWrite(LED_RIGHT, LOW);
  }
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
 * @note The `SENSOR_CURVA` parameter is not currently used in this function.
 * 
 * @return bool Returns `true` if an inversion was detected and handled, `false` otherwise.
 */
bool verifica_inversao(int SENSOR[], int SENSOR_CURVA[]) {
  return false;
  // Contadores estáticos para filtrar ruído, exigindo leituras consecutivas para mudar o estado.
  static int contador_pista_branca = 0; // Conta leituras de pista com linha branca
  static int contador_pista_preta = 0;  // Conta leituras de pista com linha preta

  // Calcula o número de sensores ativos na
  int sensores_ativos = calcula_sensores_ativos(SENSOR);

  // Condição para pista com linha branca (precisa de inversão): <= 2 sensores ativos e não é uma curva
  bool pista_branca_detectada = (sensores_ativos <= 1 && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == BRANCO);
  // Condição para pista com linha preta (NÃO precisa de inversão): >= 3 sensores ativos e não é uma curva
  bool pista_preta_detectada = (sensores_ativos >= 3 && SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO);

  if (!inversaoAtiva) { // Atualmente em pista de linha preta. Verifica se mudou para linha branca.
    if (pista_branca_detectada) {
      contador_pista_branca++;
    } else {
      contador_pista_branca = 0;
    }

    if (contador_pista_branca >= TOLERANCIA_INVERSAO) {
      inversaoAtiva = true;
      contador_pista_branca = 0; // Reseta contadores na transição
      contador_pista_preta = 0;
    }
  } else { // Atualmente em pista de linha branca. Verifica se voltou para linha preta.
    if (pista_preta_detectada) {
      contador_pista_preta++;
    } else {
      contador_pista_preta = 0;
    }

    if (contador_pista_preta >= TOLERANCIA_INVERSAO) {
      inversaoAtiva = false;
      contador_pista_branca = 0; // Reseta contadores na transição
      contador_pista_preta = 0;
    }
  }

  if (inversaoAtiva) {
    // Se a inversão estiver ativa, inverte os valores dos sensores para o resto do código.
    for (int i = 0; i < 5; i++) {
      SENSOR[i] = inverte_sensor(SENSOR[i]);
    }
    return true;
  }

  return false;
}

/**
 * @brief Handles the pedestrian crossing challenge.
 * 
 * This function simulates the robot's behavior at a pedestrian crossing.
 * It waits for a minimum required time (6 seconds) and then moves forward
 * to cross the track.
 */
void realiza_faixa_de_pedestre() {
  if (debugSD) write_sd(2); 
  // Wait for the minimum time of 6 seconds to cross
  delay(5200);

  int inicio = millis();

  while (TIMEOUT_FAIXA_PEDESTRE >= millis() - inicio) {
    ler_sensores();
    calcula_erro();
    if (erro == LINHA_NAO_DETECTADA) erro = 0;
    ajusta_movimento();
  }
}

/**
 * @brief Executes the reverse gear challenge.
 * 
 * The robot stops, moves backward for a fixed duration, stops again,
 * and then turns 90 degrees to the specified side.
 * 
 * @param lado_da_curva The side to turn towards after reversing (`SAIDA_DIREITA` or `SAIDA_ESQUERDA`).
 */
void realiza_marcha_re(int lado_da_curva) {
  if (debugSD) write_sd(7);
  stop_motors();
  delay(500);

  // 2. Execute a ré por um tempo fixo
  run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
  delay(1000);

  // 3. Pare novamente
  stop_motors();
  delay(500);

  // 4. Turn to the side of the second marker, as per the rules
  if (lado_da_curva == SAIDA_DIREITA) {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  } else {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  }

  stop_motors();
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