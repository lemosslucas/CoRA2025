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
 * Updates the global arrays `SENSOR` and `SENSOR_CURVA` with the values
 * read from the corresponding digital pins. The value `PRETO` (1) indicates
 * that the sensor has detected the line, and `BRANCO` (0) that it has not.
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
 * adding the PID value to the base speed. The `constrain` function ensures
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
 * crossing) has occurred, updating the `faixa_de_pedestre` flag. Then, it
 * calculates the error using a weighted average of the 5 line sensor readings.
 * The error indicates how far and to which side the robot is from the line's center.
 * If all sensors are on black, it considers the line lost.
 * The result is stored in the global variable `erro`.
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
int contaMarcacao(int estadoSensor, int contagemAtual, bool &jaContou) {
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

/*
*/
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
    logFile.println("Time,Error"); 
    logFile.flush(); 
    if(debugMode) Serial.println("Log file created successfully.");
  } else {
    // Se falhar mesmo com o nome curto, o problema é outro.
    if(debugMode) Serial.println("Failed to create the log file.");
  }
}

unsigned long ultimoFlush = 0;

void write_sd() {
  // Verifica se o arquivo de log está realmente aberto
  if (logFile) {
    // Escreve o tempo e o erro, separados por vírgula
    logFile.print(millis());
    logFile.print(",");
    
    // Usa println() no último dado para adicionar a quebra de linha
    logFile.println(erro); 
    
    // Força a escrita imediata para o cartão SD para evitar perda de dados
    logFile.flush(); 
    
    if (debugMode) {
      Serial.println("Log entry saved to SD.");
    }
  }
}

// Initialize variables
int marcacoesDireita = 0, marcacoesEsquerda = 0;

// Verification for marker counting, one for each side.
bool jaContouEsquerda = false, jaContouDireita = false;

// Counter for consecutive line loss and tolerance limit
int contadorLinhaPerdida = 0;

void loop() {  
  // verifica se o led ta ligado
  if (ledLigado) {
    // verifica se o led ta ligado por mais de 3 segundos
    if (millis() - tempoLedLigou >= TEMPO_MAX_LED_LIGADO) {
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, LOW);
      ledLigado = false;
    }
  }

  if (!debugMode) {
    // Calculate the car's error on the line
    calcula_erro();
    
    if (debugSD) write_sd();

    // Check if there is a 90-degree curve
    int saidaCurva = verifica_curva_90(SENSOR, SENSOR_CURVA);

    // Check if the curve was detected
    if (saidaCurva != CURVA_NAO_ENCONTRADA) { 
      // If there is a curve, store the number of markers
      while(erro != LINHA_NAO_DETECTADA) {
        // Update sensor values
        ler_sensores();
        
        marcacoesEsquerda = contaMarcacao(SENSOR_CURVA[0], marcacoesEsquerda, jaContouEsquerda);
        marcacoesDireita = contaMarcacao(SENSOR_CURVA[1], marcacoesDireita, jaContouDireita);

        // Ensure the robot stays on the line
        calcula_erro();
        calcula_PID();
        ajusta_movimento();
      }
      
      // Determine which action should be taken
      if (marcacoesEsquerda == 1 || marcacoesDireita == 1) {
        turn_90(saidaCurva);
        // Reset the number of markers
        marcacoesEsquerda = 0; jaContouEsquerda = false;
        marcacoesDireita = 0; jaContouDireita = false;
      } else if ((marcacoesEsquerda > 1 && marcacoesEsquerda <= 2) 
      || (marcacoesDireita > 1 && marcacoesDireita <= 2)) {
        saidaCurva = determina_saida_curva(marcacoesEsquerda, marcacoesDireita);
        realiza_marcha_re(saidaCurva);

        // Reset the number of markers
        marcacoesEsquerda = 0; jaContouEsquerda = false;
        marcacoesDireita = 0; jaContouDireita = false;
      } else{
        saidaCurva = determina_saida_curva(marcacoesEsquerda, marcacoesDireita);

        int numeroDeMarcas = (saidaCurva == SAIDA_ESQUERDA) ? marcacoesEsquerda : marcacoesDireita;

        realiza_rotatoria(saidaCurva, determina_saida_rotatoria(saidaCurva, numeroDeMarcas));
        
        // Reset the number of markers
        marcacoesEsquerda = 0; jaContouEsquerda = false;
        marcacoesDireita = 0; jaContouDireita = false;
      }

      stop_motors();
    } else {
      // If no curve is detected and the line is lost
      if (erro == LINHA_NAO_DETECTADA) {
        contadorLinhaPerdida++; // Increment counter for each consecutive loop without seeing the line
        if (contadorLinhaPerdida >= LIMITE_TOLERANCIA_LINHA_PERDIDA) {
          // Se o limite foi atingido, inicia a rotina de recuperação (parar e ré)
          PID = 0;
          stop_motors();

          // Verifica se é uma faixa de pedestre
          if (faixa_de_pedestre) {
            realiza_faixa_de_pedestre();
            faixa_de_pedestre = false;
            contadorLinhaPerdida = 0; // Reseta o contador
          } else {
            // Se não for faixa de pedestre, tenta reencontrar a linha dando ré
            unsigned long tempoPerdido = millis();
            bool linhaEncontradaRe = false;
            
            run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
            while (millis() - tempoPerdido < TIME_WITHOUT_LINE) {
              ler_sensores();
              if (calcula_sensores_ativos(SENSOR) > 0) {
                stop_motors();
                linhaEncontradaRe = true;
                contadorLinhaPerdida = 0; // Reseta o contador
                break;
              }
              delay(5);
            }
            // Se o tempo esgotou e não encontrou a linha, para definitivamente
            if (!linhaEncontradaRe) {
                stop_motors();
                digitalWrite(LED_LEFT, HIGH);
                digitalWrite(LED_RIGHT, HIGH);
                tempoLedLigou = millis();
                ledLigado = true;
                if (debugMode) Serial.println("Área de parada detectada. Robô parado.");
                if (debugSD) write_sd();
                while(true);
            }
          }
        } else {
          P = erroAnterior; // Usa o erro anterior como base
          I = constrain(I + P, -255, 255);
          D = 0; // Zera o derivativo para evitar movimentos bruscos
          PID = (Kp * P) + (Ki * I) + (Kd * D) + OFFSET;
          ajusta_movimento();
        }
      } else {
        // Se o robô detecta a linha normalmente, reseta o contador e segue a linha
        contadorLinhaPerdida = 0;
        calcula_PID();
        ajusta_movimento();
      }
    }
  }
  else { 
      if (debugMotor) {
        //run(velocidadeBaseDireita, velocidadeBaseEsquerda);
        //delay(3000);
        //stop_motors();
        //delay(1000);
        //run_backward(velocidadeBaseDireita, velocidadeBaseEsquerda);
        //delay(3000);
        //stop_motors();
        //delay(1000);
        turn_90(CURVA_DIREITA);
        stop_motors();
        delay(1000);
        turn_90(CURVA_ESQUERDA);
        stop_motors();
        delay(1000);
      } else {
        // Get the output of the car's data
        ler_sensores();
        imprime_serial();
      }
  }
  delay(5);
}
