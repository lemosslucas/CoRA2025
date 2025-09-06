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
 * @brief Configura e inicializa os componentes do robô.
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

  if (debugSD) setup_sd();
  // liga os leds da cara para indicar que o robô iniciou
  digitalWrite(LED_LEFT, HIGH); 
  digitalWrite(LED_RIGHT, HIGH); 
  tempoLedLigou = millis(); 
  ledLigado = true; 
}

/**
 * @brief Lê o estado digital dos sensores de linha e curva.
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
 * @brief Imprime informações de depuração no monitor serial.
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
  Serial.print("\tErro State: ");
  Serial.print(erro);
  Serial.print(" Velocidade Direita: ");
  Serial.print(velocidadeDireita); 
  Serial.print(" Velocidade Esquerda: ");
  Serial.println(velocidadeEsquerda); 
}


void setup_sd() {
  if (!SD.begin(chipSelect)) { 
    if (debugMode) Serial.println("SD Card initialization failed!");
    digitalWrite(LED_LEFT, HIGH); 
    ledLigado = true; 
    tempoLedLigou = millis(); 
    return;
  }
  
  String baseName = "L" + String((int)Kp); 
  String newFileName;
  
  for (int i = 0; i < 100; i++) {
    newFileName = baseName + "_" + String(i) + ".TXT"; 
    if (!SD.exists(newFileName)) { 
      break;
    }
  }

  if (debugMode) {
    Serial.print("Creating new log file: ");
    Serial.println(newFileName); 
  }

  logFile = SD.open(newFileName, FILE_WRITE); 
  if (logFile) {
    logFile.println("Time,Error,Challenge,Velocidade Direita,Velocidade Esquerda"); 
    logFile.flush(); 
    if(debugMode) Serial.println("Log file created successfully."); 
  } else {
    if(debugMode) Serial.println("Failed to create the log file."); 
  }
}

void write_sd(int challenge_marker = 0) {
  if (logFile) {
    logFile.print(millis()); 
    logFile.print(","); 
    logFile.print(erro); 
    logFile.print(","); 
    logFile.print(challenge_marker); 
    logFile.print(","); 
    logFile.print(velocidadeDireita); 
    logFile.print(","); 
    logFile.println(velocidadeEsquerda); 
    
    logFile.flush(); 
  }
}

/**
 * @brief Controla o movimento do robô usando uma lógica On/Off.
 */
void controlador_on_off() {
  // Atualiza a leitura dos sensores
  ler_sensores();

  // Define os estados dos sensores para facilitar a leitura do código
  // PRETO = 1 (linha detectada), BRANCO = 0
  bool sensorEsquerda = (SENSOR[1] == PRETO);
  bool sensorCentro = (SENSOR[2] == PRETO);
  bool sensorDireita = (SENSOR[3] == PRETO);

  // Lógica de controle On/Off
  if (sensorCentro) {
    // Robô está no centro da linha, segue em frente com velocidade máxima
    run(velocidadeBaseDireita, velocidadeBaseEsquerda);
    erro = 0; // Reseta o erro, pois a linha foi encontrada
  } else if (sensorEsquerda) {
    // Robô está desviando para a direita, precisa corrigir para a esquerda
    // Para isso, reduzimos a velocidade da roda esquerda
    run(velocidadeBaseDireita, 50); 
    erro = 0;
  } else if (sensorDireita) {
    // Robô está desviando para a esquerda, precisa corrigir para a direita
    // Para isso, reduzimos a velocidade da roda direita
    run(50, velocidadeBaseEsquerda);
    erro = 0;
  } else {
    // A linha não foi detectada pelos sensores principais.
    erro = LINHA_NAO_DETECTADA;
  }
}

unsigned int contadorLinhaPerdida = 0;
unsigned long tempoSemLinha = 0;
int tentativasRecuperacao = 0;
const int LIMITE_TENTATIVAS_RECUPERACAO = 3; 
unsigned long tempoUltimaRecuperacao = 0;
const unsigned long TEMPO_RESET_TENTATIVAS = 3000; 

void loop() {  
  verifica_estado_led(); 
  if (!debugMode) { 
    ler_sensores(); 
    int saidaCurva = verifica_curva_90(SENSOR, SENSOR_CURVA); 
    if (saidaCurva == CURVA_NAO_ENCONTRADA) { 
      
      controlador_on_off();

      if (erro != LINHA_NAO_DETECTADA) { 
        if (millis() - tempoUltimaRecuperacao > TEMPO_RESET_TENTATIVAS) { 
          tentativasRecuperacao = 0; 
        }
        if (debugSD) write_sd(0); 
        contadorLinhaPerdida = 0; 
      } else if (erro == LINHA_NAO_DETECTADA) { 
        if (contadorLinhaPerdida == 0) {
          tempoSemLinha = millis(); 
        }
        contadorLinhaPerdida++; 
        if (contadorLinhaPerdida > LIMITE_TOLERANCIA_LINHA_PERDIDA || millis() - tempoSemLinha > 350) { 
          stop_motors(); 
          if (tenta_recuperar_linha()) { 
            contadorLinhaPerdida = 0; 
            tentativasRecuperacao++; 
            tempoUltimaRecuperacao = millis(); 
            if (tentativasRecuperacao >= LIMITE_TENTATIVAS_RECUPERACAO) { 
              run(velocidadeBaseDireita, velocidadeBaseEsquerda); 
              delay(2000);   
                if (debugSD) write_sd(3); 
                area_de_parada(); 
            }
          } else {
            if (debugSD) write_sd(3); 
            area_de_parada(); 
          }
        }
      }
    } else if (saidaCurva != CURVA_NAO_ENCONTRADA) {
      analisa_marcacoes(); 
      if (marcacoesDireita == 1 || marcacoesEsquerda == 1) { 
        turn_90(saidaCurva); 
      } 
      stop_motors(); 
    }
  } else { 
    if (debugMotor) {
      test_motors(); 
    } else {
      ler_sensores(); 
      imprime_serial(); 
    }
  }
  delay(5);
}