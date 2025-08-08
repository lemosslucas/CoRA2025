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
 * This function analyzes the state of the main and curve sensors to identify
 * a 90-degree turn pattern and determine its direction.
 * 
 * @param SENSOR Array containing the states of the main sensors.
 * @param SENSOR_CURVA Array containing the states of the curve detection sensors.
 * 
 * @return int An integer representing the turn direction:
 * - `CURVA_ESQUERDA` if a left turn is detected.
 * - `CURVA_DIREITA` if a right turn is detected.
 * - `CURVA_EM_DUVIDA` if the turn direction is uncertain.
 * - `CURVA_NAO_ENCONTRADA` if no turn is detected.
 */

int verifica_curva_90(int SENSOR[], int SENSOR_CURVA[]) {
  // Check for a left turn
  if (SENSOR_CURVA[0] == BRANCO && SENSOR[0] == BRANCO && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO && SENSOR[4] == PRETO && SENSOR_CURVA[1] == PRETO
  || calcula_sensores_ativos(SENSOR) == 3 && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == PRETO) {
    return CURVA_ESQUERDA;
  } else if (SENSOR_CURVA[0] == PRETO && SENSOR[0] == PRETO && SENSOR[1] == PRETO && SENSOR[2] == BRANCO && SENSOR[3] == PRETO && SENSOR[4] == BRANCO && SENSOR_CURVA[1] == BRANCO) { // Check for a right turn
    return CURVA_DIREITA;
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
  // Stop the car for greater stability
  stop_motors();
  delay(200);

  if (curvaEncontrada == CURVA_ESQUERDA) {
    digitalWrite(LED_LEFT, HIGH);
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(ANGLE_CURVE);
    digitalWrite(LED_LEFT, LOW);
  } else if (curvaEncontrada == CURVA_DIREITA) {
    digitalWrite(LED_RIGHT, HIGH);
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(ANGLE_CURVE);
    digitalWrite(LED_RIGHT, LOW);
  } 
  /*
  else if (curvaEncontrada == CURVA_EM_DUVIDA) {
    // Side to be determined on competition day
    
    //turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);

    turn_until_angle(ANGLE_CURVE);
  }
  */
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
      angle_z *= 10; // Correction factor
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
  if (calcula_sensores_ativos(SENSOR) == 1) {
    // Invert the state of the sensors
    for (int i = 0; i < 5; i++) {
      SENSOR[i] = inverte_sensor(SENSOR[i]);
    }
    // Return true to indicate an inversion occurred
    return true;
  }
  // Return false as no inversion was detected
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
  // Wait for the minimum time of 6 seconds to cross
  delay(6000);
  // Move forward to cross the track
  run(255, 255);
  delay(2000); 
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
  stop_motors();
  delay(500);

  // 2. Execute a ré por um tempo fixo
  run_backward(200, 200);
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
void realiza_rotatoria(int saidaCurva, int saidaDesejada){
  // Initialize the current exit count
  int saidaAtual = 1;

  // Perform a 90-degree turn to enter the roundabout
  if (saidaCurva == CURVA_ESQUERDA) {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  } else if (saidaCurva == CURVA_DIREITA) {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
    turn_until_angle(90);
  }

  // Loop until the robot reaches the correct exit
  while(saidaAtual != saidaDesejada) {
    // Calculate error to stay on the line
    calcula_erro();
    ajusta_movimento();

    // Check which side the exit should be on (based on global variable)
    if (saida_rotatoria == SAIDA_ESQUERDA) {
      // Check for a marker on the right (indicating an exit)
      if (calcula_sensores_ativos(SENSOR) <= 3 && SENSOR_CURVA[0] == PRETO && SENSOR_CURVA[1] == BRANCO) {
        delay(200);
        // Update the current exit count
        saidaAtual++;
      }
    } else if(saida_rotatoria == SAIDA_DIREITA) {
      // Check for a marker on the left (indicating an exit)
      if (calcula_sensores_ativos(SENSOR) <= 3 && SENSOR_CURVA[0] == BRANCO && SENSOR_CURVA[1] == PRETO) {
        delay(200);
        // Update the current exit count
        saidaAtual++;
      }
    }
  }
  // Exit to the correct side of the roundabout
  if (saida_rotatoria == SAIDA_ESQUERDA) {
    turn_left(velocidadeBaseDireita, velocidadeBaseEsquerda);
  } else {
    turn_right(velocidadeBaseDireita, velocidadeBaseEsquerda);
  }
}