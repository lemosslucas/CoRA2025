/**
 * @file test_components.ino
 * @brief Suíte de testes completa para verificar os componentes do robô CoRA.
 * 
 * Este sketch permite testar os motores e os sensores de forma isolada
 * através de um menu interativo no Monitor Serial.
 * 
 * Como usar:
 * 1. Carregue este código no seu robô.
 * 2. Abra o Monitor Serial (velocidade 9600).
 * 3. Siga as instruções no menu para testar os componentes.
 */

// --- Inclusão dos arquivos do projeto e bibliotecas ---
#include "constants.h"
#include "motors.h"
#include "challenges.h"
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "CoRA2025.h"

// --- Objetos Globais necessários para os testes ---
MPU6050 mpu(Wire);
float gyro_bias_z;

// --- Funções de Teste ---

/**
 * @brief Testa os movimentos básicos dos motores.
 */
void test_motores() {
  Serial.println("\n--- INICIANDO TESTE DE MOTORES ---");
  int test_speed = 180;
  int test_delay = 1000;

  Serial.println("1. Andando para FRENTE...");
  run(test_speed, test_speed);
  delay(test_delay);
  stop_motors();
  delay(500);

  Serial.println("2. Andando para TRÁS...");
  run_backward(test_speed, test_speed);
  delay(test_delay);
  stop_motors();
  delay(500);

  Serial.println("3. Virando para a ESQUERDA...");
  turn_left(test_speed, test_speed);
  turn_until_angle(90);
  stop_motors();
  delay(500);

  Serial.println("4. Virando para a DIREITA...");
  turn_right(test_speed, test_speed);
  turn_until_angle(90);
  stop_motors();
  delay(500);

  Serial.println("--- FIM DO TESTE DE MOTORES ---");
}

/**
 * @brief Lê e exibe o estado dos sensores infravermelhos.
 */
void test_sensores() {
  Serial.println("\n--- INICIANDO TESTE DE SENSORES ---");
  Serial.println("--- FIM DO TESTE DE SENSORES ---");
  Serial.println("Pressione qualquer tecla e envie para PARAR o teste.");
  Serial.println("-------------------------------------------------");
  Serial.println("C_ESQ | S1 | S2 | S3 | S4 | S5 | C_DIR");
  Serial.println("-------------------------------------------------");

  // Limpa qualquer entrada serial antiga
  while(Serial.available()) Serial.read();

  while (!Serial.available()) {
    // Lê todos os 7 sensores
    int s_curva_esq = digitalRead(sensor0_curva_A0);
    int s1 = digitalRead(sensor1_A1);
    int s2 = digitalRead(sensor2_A2);
    int s3 = digitalRead(sensor3_A3);
    int s4 = digitalRead(sensor4_A4);
    int s5 = digitalRead(sensor5_A5);
    int s_curva_dir = digitalRead(sensor6_curva_A6);

    // Imprime os valores formatados
    Serial.print("  ");
    Serial.print(s_curva_esq);
    Serial.print("   | ");
    Serial.print(s1);
    Serial.print("  | ");
    Serial.print(s2);
    Serial.print("  | ");
    Serial.print(s3);
    Serial.print("  | ");
    Serial.print(s4);
    Serial.print("  | ");
    Serial.print(s5);
    Serial.print("  |   ");
    Serial.println(s_curva_dir);

    delay(250); // Atraso para não sobrecarregar o monitor
  }

  // Limpa a entrada que parou o teste
  while(Serial.available()) Serial.read();
  
  Serial.println("-------------------------------------------------");
  Serial.println("--- FIM DO TESTE DE SENSORES ---");
}

/**
 * @brief Testa o giroscópio (MPU6050).
 */
void test_giroscopio() {
  Serial.println("\n--- INICIANDO TESTE DO GIROSCÓPIO ---");
  
  // Calibra o giroscópio usando a função do projeto
  gyro_bias_z = calibrate_gyro();

  Serial.println("\nLendo valores do Giroscópio (Eixo Z)...");
  Serial.println("Pressione qualquer tecla e envie para PARAR o teste.");
  Serial.println("-------------------------------------------------");
  
  while(Serial.available()) Serial.read();

  while(!Serial.available()) {
    mpu.update();
    float gz = mpu.getGyroZ();
    float gz_calibrado = gz - gyro_bias_z;

    Serial.print("Raw Gz: ");
    Serial.print(gz, 4);
    Serial.print("\t | Calibrated Gz: ");
    Serial.println(gz_calibrado, 4);
    delay(100);
  }

  while(Serial.available()) Serial.read();
  Serial.println("-------------------------------------------------");
  Serial.println("--- FIM DO TESTE DO GIROSCÓPIO ---");
}

/**
 * @brief Testa o LED embutido na placa.
 */
void test_led() {
  Serial.println("\n--- INICIANDO TESTE DO LED ---");
  Serial.println("O LED da placa irá piscar 5 vezes.");
  
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }

  Serial.println("--- FIM DO TESTE DO LED ---");
}


void print_menu() {
  Serial.println("\n===== TESTES CoRA =====");
  Serial.println("Escolha uma opção:");
  Serial.println("  (1) - Testar Motores");
  Serial.println("  (2) - Testar Sensores Infravermelho");
  Serial.println("  (3) - Testar Giroscópio");
  Serial.println("  (4) - Testar LED");
  Serial.println("================================");
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();

  // Configura os motores usando a função do projeto
  setup_motor();

  // Configura os pinos dos sensores e do LED
  pinMode(sensor0_curva_A0, INPUT);
  pinMode(sensor1_A1, INPUT);
  pinMode(sensor2_A2, INPUT);
  pinMode(sensor3_A3, INPUT);
  pinMode(sensor4_A4, INPUT);
  pinMode(sensor5_A5, INPUT);
  pinMode(sensor6_curva_A6, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  print_menu();
}
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    // Limpa o buffer serial para evitar leituras extras (como \n ou \r)
    while(Serial.available()) Serial.read();

    switch(command) {
      case '1':
        test_motores();
        break;
      case '2':
        test_sensores();
        break;
      case '3':
        test_giroscopio();
        break;
      case '4':
        test_led();
        break;
    }
    
    print_menu();
  }
}

