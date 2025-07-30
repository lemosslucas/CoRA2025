#include "motors.h"
#include "challenges.h"
#include "constants.h"
#include <Wire.h>

MPU6050 mpu(Wire);

void setup() {
  // inicializacao dos sensores
  pinMode(sensor1_A1, INPUT);
  pinMode(sensor2_A2, INPUT);
  pinMode(sensor3_A3, INPUT);
  pinMode(sensor4_A4, INPUT);
  pinMode(sensor5_A5, INPUT);
  pinMode(sensor0_curva_A0, INPUT);
  pinMode(sensor6_curva_A6, INPUT);
  Wire.begin();
  mpu.begin();

  setup_motor();
  gyro_bias_z = calibrate_gyro();

  // inicializa a comunicacao serial
  Serial.begin(9600);
}

/**
 * @brief Le os estados [0, 1] detectados pelo sensor.
 * 
 * Atualiza os arrays SENSOR & SENSOR_CURVA com os valores
 * atuais detectados pelo sensor
 */
void ler_sensores() {
  SENSOR[0] = digitalRead(sensor1_A1);
  SENSOR[1] = digitalRead(sensor2_A2);
  SENSOR[2] = digitalRead(sensor3_A3);
  SENSOR[3] = digitalRead(sensor4_A4);
  SENSOR[4] = digitalRead(sensor5_A5);

  SENSOR_CURVA[0] = digitalRead(sensor0_curva_A0);
  SENSOR_CURVA[1] = digitalRead(sensor6_curva_A6);
}

/**
 * @brief Ajusta o movimento do carro
 * 
 * Apartir do valor gerado pelo PID junto com a velocidade base dos motores
 * altera a direção do movimento para permanecer na linha
 */
void ajusta_movimento() {
  // altera o valor de velocidade
  velocidadeDireita = constrain(velocidadeBaseDireita - PID, 0, 255);
  velocidadeEsquerda = constrain(velocidadeBaseEsquerda + PID, 0, 255);
  
  // envia a nova velocidade para a função andar
  run(velocidadeDireita, velocidadeEsquerda);
}

/**
 * @brief Calcula o erro do carro em relação a linha
 * 
 * Realiza uma media ponderada com os valores do sensor
 * gerando o erro do carro em relacao a linha.
 */
void calcula_erro() {
  // atualiza os valores do sensor
  ler_sensores();

  // verifica se houve inversao, sinalizando uma faixa de pedestre
  if (verifica_inversao(SENSOR, SENSOR_CURVA)) {
    faixa_de_pedestre = true;
  }

  // inicializa as variaveis para o calculo do erro
  int pesos[5] = {-2, -1, 0, 1, 2};
  int somatorioErro = 0;
  int sensoresAtivos = 0;

  // realiza um somatorio com os valores e o peso dos sensores
  for (int i = 0; i < 5; i++) {
    somatorioErro += SENSOR[i] * pesos[i];
    sensoresAtivos += SENSOR[i];
  }

  // determina o erro do carro
  if (sensoresAtivos == QUANTIDADE_TOTAL_SENSORES) {
    erro = LINHA_NAO_DETECTADA;
  } else {
    int sensoresInativos = QUANTIDADE_TOTAL_SENSORES - sensoresAtivos;
    erro = somatorioErro / sensoresInativos;
  }
}

/**
 * @brief Realiza o calculo do PID
 * 
 * A partir do erro e os valores das constantes,
 * Kp, Ki e Kd predefinidas realiza o calculo do
 * Controle Proporcional Derivativo (PID)
 */
void calcula_PID() {
  // inicializa as variaveis para o calculo
  PID = 0;
  P = erro;
  I = constrain(I + P, -255, 255);
  D = erro - erroAnterior;

  // calcula o PID
  PID = (Kp * P) + (Ki * I) + (Kd * D) + OFFSET;

  // atualiza o valor do PID
  erroAnterior = erro;
}

/**
 * @brief Imprime os dados do carro no monitor serial
 * 
 * Imprime os valores lido, PID, erro e a velocidade do carro
 */
void imprime_serial() {
  // imprime os valores do sensores
  Serial.print(SENSOR_CURVA[0]);
  Serial.print(" | ");

  for (int i = 0; i < 5; i++) {
    Serial.print(SENSOR[i]);
    Serial.print(" | ");
  }

  Serial.print(SENSOR_CURVA[1]);
  Serial.print(" | ");

  // imprime as variaveis Erro, PID, e velocidades
  Serial.print("\tErro: ");
  Serial.print(erro);
  Serial.print(" PID: ");
  Serial.print(PID);
  Serial.print(" Velocidade Direita: ");
  Serial.print(velocidadeDireita);
  Serial.print(" Velocidade Esquerda: ");
  Serial.println(velocidadeEsquerda);
}

/**
 * @brief Conta uma marcação (quadrado preto) usando detecção de borda.
 * 
 * @param estadoSensor O estado atual do sensor (PRETO ou BRANCO).
 * @param contagemAtual O valor atual do contador de marcações.
 * @param jaContou Referência a uma flag de estado para evitar contagens múltiplas.
 * @return int A nova contagem de marcações.
 */
int contaMarcacao(int estadoSensor, int contagemAtual, bool &jaContou) {
  if (estadoSensor == PRETO && !jaContou) {
    // Ativa a trava para não contar de novo
    jaContou = true; 
    return contagemAtual + 1;
  } else if (estadoSensor == BRANCO) {
    // Reseta a trava ao ver branco
    jaContou = false; 
  }
  // Retorna a contagem sem alterações
  return contagemAtual; 
}

// iniciliza as variaveis
int marcacoesDireita = 0, marcacoesEsquerda = 0;

// verificacao para a contagem de marcações, uma para cada lado.
bool jaContouEsquerda = false, jaContouDireita = false;

void loop() {  
  // calcula o erro do carro sobre a linha
  calcula_erro();

  // verifica se ha uma curva de 90
  int saidaCurva = verifica_curva_90(SENSOR, SENSOR_CURVA);

  // verifica se a curva foi detectada
  if (saidaCurva != CURVA_NAO_ENCONTRADA) { 
    // caso tenha curva armazena a quantidade de marcaçoes
    while(erro != LINHA_NAO_DETECTADA) {
      // atualiza o valor dos sensores
      ler_sensores();
      
      marcacoesEsquerda = contaMarcacao(SENSOR_CURVA[0], marcacoesEsquerda, jaContouEsquerda);
      marcacoesDireita = contaMarcacao(SENSOR_CURVA[1], marcacoesDireita, jaContouDireita);

      // garantem que o robo continue na linha
      calcula_erro();
      calcula_PID();
      ajusta_movimento();
    }
    
    // determina qual acao deve ser feita
    if (marcacoesEsquerda == 1 || marcacoesDireita == 1) {
      turn_90(saidaCurva);

      // reseta o numero de marcacaoes
      marcacoesEsquerda = 0; jaContouEsquerda = false;
      marcacoesDireita = 0; jaContouDireita = false;
    } else if ((marcacoesEsquerda > 1 && marcacoesEsquerda <= 2) 
    || (marcacoesDireita > 1 && marcacoesDireita <= 2)) {
      saidaCurva = determina_saida_curva(marcacoesEsquerda, marcacoesDireita);
      realiza_marcha_re(saidaCurva);

      // reseta o numero de marcacaoes
      marcacoesEsquerda = 0; jaContouEsquerda = false;
      marcacoesDireita = 0; jaContouDireita = false;
    } else{
      saidaCurva = determina_saida_curva(marcacoesEsquerda, marcacoesDireita);

      int numeroDeMarcas = (saidaCurva == SAIDA_ESQUERDA) ? marcacoesEsquerda : marcacoesDireita;

      realiza_rotatoria(determina_saida_rotatoria(saidaCurva, numeroDeMarcas));
      
      // reseta o numero de marcacaoes
      marcacoesEsquerda = 0; jaContouEsquerda = false;
      marcacoesDireita = 0; jaContouDireita = false;
    }

    stop_motors();
  } else {
    // caso nao detectado curva e tenha perdido a linha
    if (erro == LINHA_NAO_DETECTADA) {      
      PID = 0;
      stop_motors();

      // verifica se é uma faixa de pedestre
      if (faixa_de_pedestre) {
        realiza_faixa_de_pedestre();
        faixa_de_pedestre = false;
      } else {
        // Tenta dar ré até encontrar a linha ou atingir o tempo limite.
        unsigned long tempoPerdido = millis();
        
        // Começa a dar ré
        run_backward(200, 200); 

        while (millis() - tempoPerdido < TIME_WITHOUT_LINE) {
          ler_sensores();
          if (calcula_sensores_ativos(SENSOR) > 0) {
            // Linha encontrada Para a ré e sai do loop.
            break;
          }
          delay(10);
        }

        stop_motors();
        
        // Se saiu do loop porque o tempo esgotou, para permanentemente.
        if (millis() - tempoPerdido >= TIME_WITHOUT_LINE) {
          Serial.println("Área de parada detectada. Robô parado.");
          while(true);
        }
      }
    } else {
      // caso o carro detecte a linha ele segue a linha
      calcula_PID();
      ajusta_movimento();
    }
  }

  // obtem a saida dos dados do carro
  //imprime_serial(); 
  delay(5);
}
