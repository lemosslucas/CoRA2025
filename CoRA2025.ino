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


void loop() {
  // iniciliza as variaveis
  int marcacoesDireita = 0, marcacoesEsquerda = 0;
  
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
      
      // atualiza os valores de identificacao
      if (SENSOR_CURVA[0] == BRANCO) {
        marcacoesEsquerda++;
        historico_curva[i] = SAIDA_ESQUERDA;
      } else if (SENSOR_CURVA[1] == BRANCO) {
        marcacoesDireita++;
        historico_curva[i] = SAIDA_DIREITA;
      }

      i++;
      // garantem que o robo continue na linha
      calcula_erro();
      calcula_PID();
      ajusta_movimento();
    }
    
    // determina qual acao deve ser feita
    if (marcacoesEsquerda == 1 || marcacoesDireita == 1) {
      turn_90(saidaCurva);
    } else if ((marcacoesEsquerda > 1 && marcacoesEsquerda <= 2) 
    || (marcacoesDireita > 1 && marcacoesDireita <= 2)) {
      saidaCurva = determina_saida_curva(marcacoesEsquerda, marcacoesDireita);
      realiza_marcha_re(saidaCurva);
    } else{
      saidaCurva = determina_saida_curva(marcacoesEsquerda, marcacoesDireita);

      int numeroDeMarcas = if (saidaCurva == SAIDA_ESQUERDA) {
        marcacoesEsquerda;
      } else {
        marcacoesDireita;
      }

      realiza_rotatoria(determina_saida_rotatoria(saidaCurva, numeroDeMarcas));
    }

    stop_motors();
  } else {
    // caso nao detectado curva e tenha perdido a linha
    if (erro == LINHA_NAO_DETECTADA) {
      // atualiza o numero de sensores ativos
      int sensoresAtivos = calcula_sensores_ativos(SENSOR);
      PID = 0;
      stop_motors();

      // verifica se é uma faixa de pedestre
      if (faixa_de_pedestre == true) {
        realiza_faixa_de_pedestre();
        faixa_de_pedestre = false;
      } else {
        // garante que o robo fica parado na linha
        delay(DELAY_LOST_LINE);

        // realiza uma re ate que volte para a linha
        while(sensoresAtivos == QUANTIDADE_TOTAL_SENSORES) {
          ler_sensores();
          sensoresAtivos = calcula_sensores_ativos(SENSOR);
          run_backward(255, 255);
        }

        stop_motors();
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
