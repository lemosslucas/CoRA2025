#include "challenges.h"
#include "motors.h"
#include "constants.h"

/**
 * @brief Calcula o numero de sensores ativos
 * 
 * @param SENSOR Array com as saidas dos sensores
 * 
 * @note Um sensor esta no estado ativo é quando a saida do sensor
 * for igual a 1 = Preto
 * 
 * @return int sensoresAtivos- O numero de sensores ativos
 */
int calcula_sensores_ativos(int SENSOR[]) {
  int sensoresAtivos = 0;
  // calcula o numero de sensores ativos
  for(int i = 0; i < 5; i++) {
    sensoresAtivos += SENSOR[i];
  }
  
  // retorna o numero de sensores ativos
  return sensoresAtivos;
}

/**
 * @brief Determina se o carro deve realizar uma curva de 90 graus.
 * 
 * Esta função analisa o estado dos sensores principais e dos sensores de curva 
 * para identificar se o carro deve realizar uma curva de 90 graus e, em caso 
 * positivo, determina o lado da curva.
 * 
 * @param SENSOR Array contendo os estados dos sensores principais.
 * @param SENSOR_CURVA Array contendo os estados dos sensores responsáveis pela 
 * detecção da curva de 90 graus.
 * 
 * @return int Representa o lado da curva:
 * - CURVA_ESQUERDA se a curva for para a esquerda.
 * - CURVA_DIREITA se a curva for para a direita.
 * - CURVA_EM_DUVIDA se não houver certeza sobre o lado da curva.
 * - CURVA_NAO_ENCONTRADA se nenhuma curva for detectada.
 */

int verifica_curva_90(int SENSOR[], int SENSOR_CURVA[]) {
  // verifica se a curva é para esquerda
  if (SENSOR_CURVA[0] == BRANCO && SENSOR[0] == BRANCO && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO && SENSOR[4] == PRETO && SENSOR_CURVA[1] == PRETO
  || calcula_sensores_ativos(SENSOR) == 3 && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == PRETO) {
    return CURVA_ESQUERDA;
  } else if (SENSOR_CURVA[0] == PRETO && SENSOR[0] == PRETO && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO && SENSOR[4] == BRANCO && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_DIREITA;
  } else if (SENSOR_CURVA[0] == BRANCO && SENSOR[0] == BRANCO && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO && SENSOR[4] == BRANCO && SENSOR_CURVA[1] == BRANCO) {
    return CURVA_EM_DUVIDA;
  }

  return CURVA_NAO_ENCONTRADA;
}

/**
 * @brief Realiza a curva de 90
 * 
 * @param curvaEncontrada - lado que deve ser feito a curva
 * 
 * A partir do lado determinado da curva a função realiza o movimento de curva
 * de 90.
 */
void turn_90(int curvaEncontrada) {
  // para o carro para maior estabilidade
  stop_motors();
  delay(200);

  // verifica qual lado deve ser feito a curva
  if (curvaEncontrada == CURVA_ESQUERDA) {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(ANGLE_CURVE);
  } else if (curvaEncontrada == CURVA_DIREITA) {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(ANGLE_CURVE);
  } else if (curvaEncontrada == CURVA_EM_DUVIDA) {
    // lado determinado no dia da prova

    //turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);

    turn_until_angle(ANGLE_CURVE);
  }
}


/**
 * @brief Mede o desvio (bias) do giroscópio no eixo Z com o robô parado.
 * 
 * @param mpu Referência para o objeto do giroscópio (MPU).
 * @param samples O número de amostras a serem coletadas para a calibração.
 * @return float O valor do desvio (bias) calculado para o eixo Z.
 */
float calibrate_gyro(int samples = 400) {
    Serial.println("Calibrando o giroscópio... Mantenha o robô parado.");
    
    float sum_gz = 0;
    for (int i = 0; i < samples; i++) {
        mpu.update(); 
        sum_gz += mpu.getGyroZ();
        delay(10); 
    }
            
    float bias_gz = sum_gz / samples;
    
    Serial.print("Calibração concluída. Bias do Giroscópio (Gz) = ");
    Serial.println(bias_gz, 4);

    return bias_gz;
}

void turn_until_angle(int target_angle) {
  unsigned long previous_time = millis();
  float angle_z = 0;

  while (abs(angle_z) < target_angle) {
      mpu.update(); 
      float angular_velocity_z = mpu.getGyroZ() - gyro_bias_z; 

      unsigned long current_time = millis();
      float delta_time = (current_time - previous_time) / 1000.0; 
      previous_time = current_time;

      angle_z += angular_velocity_z * delta_time;

      delay(10);
  }
  stop_motors();

  Serial.println("Rotation finished");
}


/**
 * @brief Inverte o valor obtido pelos sensores
 * 
 * @param int sensor - saida dos sensores
 * 
 * Responsavel por garantir que o carro siga a linha quando houver
 * a inversao de cores preto/branco -> branco/preto aplicando a 
 * logica de uma porta logica NOT.
 */
int inverte_sensor(int sensor){
  if (sensor == 1){ 
    return 0;
  } 
  return 1;
}

/**
 * @brief Verifica se houve uma inversão nas cores da pista.
 * 
 * Esta função analisa o estado dos sensores principais e dos sensores de curva 
 * para determinar se houve uma inversão nas cores da pista (preto/branco). 
 * Caso detectada, a função inverte os valores dos sensores principais para 
 * que o carro possa continuar seguindo a linha.
 * 
 * @param SENSOR Array contendo os estados dos sensores principais.
 *               Cada posição representa o estado de um sensor.
 * @param SENSOR_CURVA Array contendo os estados dos sensores de curva.
 *                     Usado para complementar a verificação de inversão.
 * 
 * @return bool Retorna:
 * - `true` se uma inversão foi detectada e os sensores principais foram invertidos.
 * - `false` se nenhuma inversão foi detectada.
 */
bool verifica_inversao(int SENSOR[], int SENSOR_CURVA[]) {
  if (calcula_sensores_ativos(SENSOR) == 1) {
    // inverte os estados dos sensores
    for (int i = 0; i < 5; i++) {
      SENSOR[i] = inverte_sensor(SENSOR[i]);
    }
    // retorna true para indiciar que houve inversão
    return true;
  }
  // retorna false ja que não houve a inversão
  return false;
}

/**
 * @brief Realiza o movimento de atraversar a faixa de pedestre
 * 
 * Esta função simula o comportamento de um robô ao atravessar uma faixa 
 * de pedestres. Primeiro, ele espera o tempo mínimo necessário para garantir 
 * segurança antes de atravessar, depois avança para atravessar a pista.
 */
void realiza_faixa_de_pedestre() {
  // espera o tempo minimo de 5seg para poder atravessar
  delay(6000);
  // anda para frente para atravessar a pista
  run(255, 255);
  delay(2000); 
}

void realiza_marcha_re(int lado_da_curva) {
  stop_motors();
  delay(500);

  // 2. Execute a ré por um tempo fixo
  run_backward(200, 200);
  delay(1000);

  // 3. Pare novamente
  stop_motors();
  delay(500);

  // 4. Vire para o lado da segunda marca, conforme o edital
  if (lado_da_curva == SAIDA_DIREITA) {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  } else {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  }

  stop_motors();
}

int determina_saida_curva(int marcacoesEsquerda, int marcacoesDireita) {
  if (marcacoesDireita < marcacoesEsquerda) {
    return SAIDA_DIREITA;
  } else {
    return SAIDA_ESQUERDA
  }
}


/**
 * @brief Determina o lado da saída em uma rotatória baseado no número de marcações detectadas.
 * 
 * @param marcacoesEsquerda Armazena as marcacoes a esquerda
 * @param marcacoesDireita Armazena as marcacoes a direita
 * 
 * A partir do numero de marcações a função atualiza o lado correto para
 * a saida do carro na rotatoria.
 * 
 * @return int saidaDesejada 
 */
int determina_saida_rotatoria(int saidaCurva, int numeroDeMarcas) {
  if (saidaCurva == CURVA_ESQUERDA) {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  } else if (saidaCurva == CURVA_DIREITA) {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  }

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
 * @brief Realiza a saida na rotatoria
 * 
 * @param saidaDesejada Numero que deve ser feito a saida da rotatoria
 * 
 * O carro segue o percurso ate chegar na `saidaDesejada`
 */
void realiza_rotatoria(int saidaDesejada){
  // inicializa a saida atual
  int saidaAtual = 1;

  // loop para que o carro sai apenas na saida correta
  while(saidaAtual != saidaDesejada) {
    // calcula o erro para manter o carro na linha
    calcula_erro();
    ajusta_movimento();

    // verifica qual lado deve ser feito a saida
    if (saida_rotatoria == SAIDA_ESQUERDA) {
      // verifica se ha alguma marcacao na direita
      if (calcula_sensores_ativos(SENSOR) <= 3 && SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == BRANCO) {
        delay(200);
        // atualiza o valor da saida atual
        saidaAtual++;
      }
    } else if(saida_rotatoria == SAIDA_DIREITA) {
      // verifica se ha alguma marcacao na esquerda
      if (calcula_sensores_ativos(SENSOR) <= 3 && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == PRETO) {
        delay(200);
        // atualiza o valor da saida atual
        saidaAtual++;
      }
    }
  }
  
  // sai para o lado correto da rotatoria
  if (saida_rotatoria == SAIDA_ESQUERDA) {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
  } else {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
  }
}

 