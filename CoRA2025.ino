#include "motors.h"
#include "challenges.h"
#include "constants.h"
#include "CoRA2025.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

MPU6050 mpu(Wire);
File logFile;

/**
 * @brief Configures and initializes the robot's components.
 * 
 * This function runs once at startup. It configures the sensor and LED
 * pins as input/output, initializes I2C communication (Wire) for the
 * gyroscope (MPU6050), calibrates the gyroscope to get the bias,
 * sets up the motors, and initializes serial communication.
 */
void setup() {
  // Initialize serial communication
  if (debugMode) Serial.begin(9600);
  if (debugSD) setup_sd();
  
  // Sensor initialization
  pinMode(sensor_esquerda, INPUT);
  pinMode(sensor_esquerda_central, INPUT);
  pinMode(sensor_central, INPUT);
  pinMode(sensor_direita_central, INPUT);
  pinMode(sensor_direita, INPUT);
  pinMode(sensor_curva_esquerda, INPUT);
  pinMode(sensor_curva_direita, INPUT);
  pinMode(LEDS, OUTPUT);

  // Gyroscope initialization
  Wire.begin();
  mpu.begin();

  // liga os motores
  setup_motor();

  // Calibrate the gyroscope
  gyro_bias_z = calibrate_gyro();
  
  // liga os leds da cara para indicar que o robô iniciou
  digitalWrite(LEDS, HIGH);
  tempoLedLigou = millis();
  ledLigado = true;
}

/**
 * @brief Reads the digital state of the line and curve sensors.
 * 
 * Updates the global arrays SENSOR and SENSOR_CURVA with the values
 * read from the corresponding digital pins. The value PRETO (1) indicates
 * that the sensor has detected the line, and BRANCO (0) that it has not.
 */
void ler_sensores() {
  SENSOR[0] = digitalRead(sensor_esquerda);
  SENSOR[1] = digitalRead(sensor_esquerda_central);
  SENSOR[2] = digitalRead(sensor_central);
  SENSOR[3] = digitalRead(sensor_direita_central);
  SENSOR[4] = digitalRead(sensor_direita);

  SENSOR_CURVA[0] = digitalRead(sensor_curva_esquerda);
  SENSOR_CURVA[1] = digitalRead(sensor_curva_direita);
}

/**
 * @brief Adjusts the motor speeds based on the PID value.
 * 
 * Calculates the speed for each motor (right and left) by subtracting and
 * adding the PID value to the base speed. The constrain function ensures
 * that the speed values remain within the valid range (0-255).
 * It then drives the motors with the new speeds.
 */
void ajusta_movimento() { 
  // Change the speed value
  velocidadeDireita = constrain(velocidadeBaseDireita - PID, 0, 255);
  velocidadeEsquerda = constrain(velocidadeBaseEsquerda + PID, 0, 255);
  // Send the new speed to the run function
  run(velocidadeDireita, velocidadeEsquerda);
}

/**
 * @brief Calculates the robot's position error relative to the line.
 * 
 * First, it reads the sensors and checks if a track inversion (pedestrian
 * crossing) has occurred, updating the faixa_de_pedestre flag. Then, it
 * calculates the error using a weighted average of the 5 line sensor readings.
 * The error indicates how far and to which side the robot is from the line's center.
 * If all sensors are on black, it considers the line lost.
 * The result is stored in the global variable erro.
 */
void calcula_erro() {
  // Update sensor values
  ler_sensores();

  // Check for inversion, signaling a pedestrian crossing
  verifica_inversao(SENSOR, SENSOR_CURVA);

  if (inversao_finalizada) {
    faixa_de_pedestre = true;
  }

  // Initialize variables for error calculation
  int pesos[5] = {-3, -1, 0, 1, 3};
  int somatorioErro = 0;
  int sensoresAtivos = 0;

  // Perform a summation with the sensor values and weights
  for (int i = 0; i < 5; i++) {
    somatorioErro += SENSOR[i] * pesos[i];
    sensoresAtivos += SENSOR[i];
  }

  // Determine the car's error
  if (sensoresAtivos == QUANTIDADE_TOTAL_SENSORES) {
    erro = LINHA_NAO_DETECTADA;
  } else {
    int sensoresInativos = QUANTIDADE_TOTAL_SENSORES - sensoresAtivos;
    erro = somatorioErro / sensoresInativos;
  }
}

/**
 * @brief Calculates the PID (Proportional-Integral-Derivative) control value.
 * 
 * Uses the current error to calculate the three control components:
 * - Proporcional (P): Proporcional ao erro atual.
 * - Integral (I): Acumula o erro ao longo do tempo para corrigir desvios persistentes.
 * - Derivativo (D): Responde à taxa de variação do erro para amortecer oscilações.
 * 
 * The final PID value is the weighted sum of these components and is used to
 * adjust the motor speeds.
 */
void calcula_PID() {
  // Initialize variables for calculation
  PID = 0;
  P = erro;
  I = constrain(I + P, -255, 255);
  D = erro - erroAnterior;

  // Calculate PID
  PID = (Kp * P) + (Ki * I) + (Kd * D) + OFFSET;

  // Update the previous error value
  erroAnterior = erro;
}

/**
 * @brief Prints debugging information to the serial monitor.
 * 
 * Sends the state of the curve and line sensors, the calculated error value,
 * the PID output value, and the resulting speeds for the right and left motors
 * to the serial. Useful for calibrating and monitoring the robot's behavior.
 */
void imprime_serial() {
  // Print sensor values
  Serial.print(SENSOR_CURVA[0]);
  Serial.print(" | ");

  for (int i = 0; i < 5; i++) {
    Serial.print(SENSOR[i]);
    Serial.print(" | ");
  }

  Serial.print(SENSOR_CURVA[1]);
  Serial.print(" | ");

  // Print Error, PID, and speed variables
  Serial.print("\tErro: ");
  calcula_erro();
  Serial.print(erro);
  Serial.print(" PID: ");
  calcula_PID();
  Serial.print(PID);
  Serial.print(" Velocidade Direita: ");
  Serial.print(velocidadeDireita);
  Serial.print(" Velocidade Esquerda: ");
  Serial.println(velocidadeEsquerda);
  //turn_until_angle(90);

}


void setup_sd() {
  if (!SD.begin(chipSelect)) {
    if (debugMode) Serial.println("SD Card initialization failed!");
    return;
  }

  char newFileName[16]; 

  // Procura o primeiro índice disponível
  for (int i = 0; i < 100; i++) {
    // Formata o nome do arquivo de forma segura usando snprintf
    snprintf(newFileName, sizeof(newFileName), "L%d_%d.TXT", (int)Kp, i);
    
    // Verifica se o arquivo NÃO existe
    if (!SD.exists(newFileName)) {
      break; // Encontrou um nome disponível
    }
  }

  if (debugMode) {
    Serial.print("Creating new log file: ");
    Serial.println(newFileName);
  }

  // Abre o novo arquivo para escrita
  logFile = SD.open(newFileName, FILE_WRITE);

  if (logFile) {
    //logFile.println("Time,Error,Challenge,Inclinacao,Velocidade Direita,Velocidade Esquerda, sc0, s0, s1, s2, s3, s3, sc1"); 
    logFile.println("Time,Error,Challenge,MarcacaoDireita,MarcacaoEsquerda,T_Dir,T_Esq,Velocidade Direita,Velocidade Esquerda,sc0,s0,s1,s2,s3,s4,sc1"); 
    logFile.flush();
    if(debugMode) Serial.println("Log file created successfully.");
  } else {
    if(debugMode) Serial.println("Failed to create the log file.");
  }
}

void write_sd(int challenge_marker = 0) {
  // Verifica se o arquivo de log está realmente aberto
  if (logFile) {
    // Escreve tempo, erro e marcador de desafio
    logFile.print(millis());
    logFile.print(",");
    logFile.print(erro);
    logFile.print(",");
    logFile.print(challenge_marker);
    logFile.print(",");
    logFile.print(marcacoesDireita);
    logFile.print(",");
    logFile.print(marcacoesEsquerda);
    logFile.print(",");

    logFile.print(tempoMarcacaoDireita);
    logFile.print(",");
    logFile.print(tempoMarcacaoEsquerda);
    logFile.print(",");

    logFile.print(velocidadeDireita);
    logFile.print(",");
    logFile.print(velocidadeEsquerda);

    // Adiciona valores dos 5 sensores da linha
    for (int i = 0; i < 5; i++) {
      logFile.print(",");
      logFile.print(SENSOR[i]);
    }

    // Adiciona sensores de curva
    for (int i = 0; i < 2; i++) {
      logFile.print(",");
      logFile.print(SENSOR_CURVA[i]);
    }
    // Quebra de linha no final
    logFile.println();

    // Força escrita imediata
    logFile.flush(); 
    
    if (debugMode) {
      if (challenge_marker != 0) Serial.println("Challenge event logged to SD.");
    }
  }
}


unsigned int contadorLinhaPerdida = 0;
unsigned long tempoSemLinha = 0;
int tentativasRecuperacao = 0;
const int LIMITE_TENTATIVAS_RECUPERACAO = 3;
unsigned long tempoUltimaRecuperacao = 0;
const unsigned long TEMPO_RESET_TENTATIVAS = 3000;

unsigned int qnt_fim_inversao =0;

/**
 * @brief Aligns the robot's path when only one curve sensor detects the line.
 * * This function is designed to make gentle corrections to the robot's alignment.
 * If the left curve sensor detects the line, it indicates the robot is drifting
 * to the right. To correct this, the function turns off the left motor and keeps
 * the right motor running, guiding the robot back to the left.
 * Conversely, if the right curve sensor detects the line, the right motor is
 * turned off to guide the robot back to the right.
 * * @param SENSOR_CURVA Array containing the states of the curve sensors.
 * @return bool Returns true if an alignment adjustment was made, otherwise false.
 */
bool alinha_pela_curva(int SENSOR_CURVA[]) {
  bool alinhamento_ativo = false;

  // Verifica se APENAS o sensor de curva esquerdo detecta a linha (preto)
  if (SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == BRANCO) {
    // O robô está se desviando para a direita, então precisa virar à esquerda.
    // Para isso, desligamos o motor esquerdo e mantemos o direito funcionando.
    run(velocidadeBaseDireita, 0); 
    alinhamento_ativo = true;
    if (debugSD) write_sd(16); // Log para indicar alinhamento à esquerda
  }
  // Verifica se APENAS o sensor de curva direito detecta a linha (preto)
  else if (SENSOR_CURVA[1] == PRETO && SENSOR_CURVA[0] == BRANCO) {
    // O robô está se desviando para a esquerda, então precisa virar à direita.
    // Para isso, desligamos o motor direito e mantemos o esquerdo funcionando.
    run(0, velocidadeBaseEsquerda);
    alinhamento_ativo = true;
    if (debugSD) write_sd(17); // Log para indicar alinhamento à direita
  }

  return alinhamento_ativo;
}

void loop() {  
  // verifica o estado do led
  verifica_estado_led();

  if (!debugMode) {
    if (arrancadaMode) {
      ler_sensores();
      calcula_erro();
      calcula_PID();
      ajusta_movimento();
      if (debugSD) write_sd(0);
    } else {
      ler_sensores();
      int posicao = calcula_posicao(SENSOR);

      if (posicao != 0) {
        ultima_posicao_linha = posicao;  // guarda última leitura válida
      }
      // verifica se tem uma curva de 90
      int saidaCurva = verifica_curva_90(SENSOR, SENSOR_CURVA);
      if (!inversaoAtiva){
        if (saidaCurva != CURVA_NAO_ENCONTRADA && (millis() - tempoUltimaCurva < DEBOUNCE_TEMPO_CURVA)) {
          saidaCurva = CURVA_NAO_ENCONTRADA; 
        }
      }

      if (faixa_de_pedestre) {
        // filtro de ruido
        if (calcula_sensores_ativos(SENSOR) == QUANTIDADE_TOTAL_SENSORES) {
          qnt_fim_inversao += 1;
        }

        if (qnt_fim_inversao >= 3) {
          if (debugSD) write_sd(2);
          realiza_faixa_de_pedestre();
          faixa_de_pedestre = false;
        }
      }
      
      if (saidaCurva == CURVA_NAO_ENCONTRADA) {
        calcula_erro();
        
        if (erro != LINHA_NAO_DETECTADA) {
          bool alinhamento_ocorreu = alinha_pela_curva(SENSOR_CURVA);

          // Se o alinhamento NÃO ocorreu, segue a lógica PID normal.
          if (!alinhamento_ocorreu) {
            calcula_PID();
            if ((SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO) || inversaoAtiva) {
              ajusta_movimento();
            }
          }
          
          // segue normalmente
          calcula_PID();
          if ((SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == PRETO) || inversaoAtiva) {
            ajusta_movimento();
          }

          if (millis() - tempoUltimaRecuperacao > TEMPO_RESET_TENTATIVAS) {
            tentativasRecuperacao = 0;
          }

          if (debugSD) write_sd(0);
          contadorLinhaPerdida = 0; // reset se achou a linha
        } else if (erro == 10) {
          // perdeu a linha
          if (contadorLinhaPerdida == 0) {
            tempoSemLinha = millis(); // marca quando perdeu
          }
          contadorLinhaPerdida++;

          // verifica se a perda já é relevante (por contagem OU tempo)
          if (contadorLinhaPerdida > LIMITE_TOLERANCIA_LINHA_PERDIDA || millis() - tempoSemLinha > 1000) {
                
            stop_motors();

            // tenta recuperar a linha
            if (tenta_recuperar_linha()) {
              contadorLinhaPerdida = 0; // recuperou com sucesso
              tentativasRecuperacao++;
              tempoUltimaRecuperacao = millis();
              
              if (tentativasRecuperacao >= LIMITE_TENTATIVAS_RECUPERACAO) { 
                // já tentou muitas vezes → para definitivo
                erro = erroAnterior;
                if (debugSD) write_sd(3);
                area_de_parada();
              }
            } else {
              erro = erroAnterior;
              if (debugSD) write_sd(3); // Log challenge 3: Stop area
              area_de_parada(); // não conseguiu → para
            }
          }
        }
      } else if (saidaCurva != CURVA_NAO_ENCONTRADA) {
        // evita ligar no cruzamento
        if (calcula_sensores_ativos(SENSOR) > 1) {
          if (!inversaoAtiva) {
            marcacoesDireita = 0;
            marcacoesEsquerda = 0;
            
            //analisa_marcacoes();
            
            if (marcacoesDireita == 1 && marcacoesEsquerda == 1 ) {
              unsigned long deltaTempo;
              if (tempoMarcacaoDireita > tempoMarcacaoEsquerda) {
                deltaTempo = tempoMarcacaoDireita - tempoMarcacaoEsquerda;
              } else {
                deltaTempo = tempoMarcacaoEsquerda - tempoMarcacaoDireita;
              }

              if (deltaTempo >= TOLERANCIA_TEMPO_SIMULTANEO) {
                saidaCurva = (tempoMarcacaoDireita < tempoMarcacaoEsquerda) ? CURVA_DIREITA : CURVA_ESQUERDA;
                realiza_marcha_re(saidaCurva);
              } else {
                turn_90(CURVA_EM_DUVIDA); // ta invertido nao sei porque
                if (debugSD) write_sd(6);
              }
            } else if (marcacoesDireita >= 1 || marcacoesEsquerda >= 1) {
              turn_90(saidaCurva);
            }
          }
          
          stop_motors();
        }
      }
    }
  } else { 
    if (debugMotor) {
      test_motors();
    } else {
      // Get the output of the car's data
      ler_sensores();
      imprime_serial();
    }
  }

  delay(5);
}