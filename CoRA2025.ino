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

  // Sensor initialization
  pinMode(sensor_esquerda, INPUT);
  pinMode(sensor_esquerda_central, INPUT);
  pinMode(sensor_central, INPUT);
  pinMode(sensor_direita_central, INPUT);
  pinMode(sensor_direita, INPUT);
  pinMode(sensor_curva_esquerda, INPUT);
  pinMode(sensor_curva_direita, INPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  // Gyroscope initialization
  Wire.begin();
  mpu.begin();

  // liga os motores
  setup_motor();

  // Calibrate the gyroscope
  gyro_bias_z = calibrate_gyro();
  //mpu.calcGyroOffsets(true);

  if (debugSD) setup_sd();
  
  // liga os leds da cara para indicar que o robô iniciou
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
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
  if (verifica_inversao(SENSOR, SENSOR_CURVA)) {
    faixa_de_pedestre = true;
  }

  // Initialize variables for error calculation
  int pesos[5] = {-2, -1, 0, 1, 2};
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
    digitalWrite(LED_LEFT, HIGH);
    ledLigado = true;
    tempoLedLigou = millis();
    return;
  }
  
  String baseName = "L" + String((int)Kp); // Ex: "L100"
  String newFileName;
  
  // 2. Procure o primeiro índice disponível
  for (int i = 0; i < 100; i++) {
    // Formato final será algo como "L100_0.TXT" (7 caracteres + extensão)
    newFileName = baseName + "_" + String(i) + ".TXT"; 
    
    // 3. Verifica se o arquivo NÃO existe
    if (!SD.exists(newFileName)) {
      break; 
    }
  }

  if (debugMode) {
    Serial.print("Creating new log file: ");
    Serial.println(newFileName);
  }

  // 4. Abre o novo arquivo para escrita.
  logFile = SD.open(newFileName, FILE_WRITE);

  if (logFile) {
    logFile.println("Time,Error,Challenge,Velocidade Direita,Velocidade Esquerda"); 
    logFile.flush(); 
    if(debugMode) Serial.println("Log file created successfully.");
  } else {
    // Se falhar mesmo com o nome curto, o problema é outro.
    if(debugMode) Serial.println("Failed to create the log file.");
  }
}

void write_sd(int challenge_marker = 0) {
  // Verifica se o arquivo de log está realmente aberto
  if (logFile) {
    // Escreve o tempo, o erro e o marcador de desafio, separados por vírgula
    logFile.print(millis());
    logFile.print(",");
    logFile.print(erro);
    logFile.print(",");
    logFile.print(challenge_marker);
    logFile.print(",");
    logFile.print(velocidadeDireita);
    logFile.print(",");
    logFile.println(velocidadeEsquerda);

    // Usa println() no último dado para adicionar a quebra de linha
    
    // Força a escrita imediata para o cartão SD para evitar perda de dados
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

void loop() {  
  // verifica o estado do led
  verifica_estado_led();

  if (!debugMode) {
    ler_sensores();
    // verifica se tem uma curva de 90
    int saidaCurva = verifica_curva_90(SENSOR, SENSOR_CURVA);
    
    if (saidaCurva == CURVA_NAO_ENCONTRADA) {
      calcula_erro();
      
      if (erro != LINHA_NAO_DETECTADA) {
        // segue normalmente
        calcula_PID();
        ajusta_movimento();

        if (millis() - tempoUltimaRecuperacao > TEMPO_RESET_TENTATIVAS) {
          tentativasRecuperacao = 0;
        }

        if (debugSD) write_sd(0);
        contadorLinhaPerdida = 0; // reset se achou a linha
      } else if (erro == LINHA_NAO_DETECTADA) {
        // perdeu a linha
        if (contadorLinhaPerdida == 0) {
          tempoSemLinha = millis(); // marca quando perdeu
        }
        contadorLinhaPerdida++;

        // verifica se a perda já é relevante (por contagem OU tempo)
        if (contadorLinhaPerdida > LIMITE_TOLERANCIA_LINHA_PERDIDA || millis() - tempoSemLinha > 350) {
              
          stop_motors();

          // tenta recuperar a linha
          if (tenta_recuperar_linha()) {
            contadorLinhaPerdida = 0; // recuperou com sucesso
            tentativasRecuperacao++;
            tempoUltimaRecuperacao = millis();
            
            if (tentativasRecuperacao >= LIMITE_TENTATIVAS_RECUPERACAO) {
              run(velocidadeBaseDireita, velocidadeBaseEsquerda);
                delay(2000);  
                // já tentou muitas vezes → para definitivo
                if (debugSD) write_sd(3);
                area_de_parada();
            }
          } else {
            if (debugSD) write_sd(3); // Log challenge 3: Stop area
            area_de_parada(); // não conseguiu → para
          }
        }
      }
    } else if (saidaCurva != CURVA_NAO_ENCONTRADA) {
      analisa_marcacoes();

      if (marcacoesDireita == 1 || marcacoesEsquerda == 1) {
        turn_90(saidaCurva);
      } 

      // implementacao posterior da rotatoria e da marca re
      //
      //

      stop_motors();
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