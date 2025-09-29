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
  pinMode(left_sensor_pin, INPUT);
  pinMode(left_central_sensor_pin, INPUT);
  pinMode(central_sensor_pin, INPUT);
  pinMode(right_central_sensor_pin, INPUT);
  pinMode(right_sensor_pin, INPUT);
  pinMode(left_curve_sensor_pin, INPUT);
  pinMode(right_curve_sensor_pin, INPUT);
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
 * The value BLACK (1) indicates the sensor has detected the line, and WHITE (0) that it has not.
 */
void read_sensors() {
  SENSOR[0] = digitalRead(left_sensor_pin);
  SENSOR[1] = digitalRead(left_central_sensor_pin);
  SENSOR[2] = digitalRead(central_sensor_pin);
  SENSOR[3] = digitalRead(right_central_sensor_pin);
  SENSOR[4] = digitalRead(right_sensor_pin);

  CURVE_SENSORS[0] = digitalRead(left_curve_sensor_pin);
  CURVE_SENSORS[1] = digitalRead(right_curve_sensor_pin);
}

/**
 * @brief Adjusts the motor speeds based on the PID value.
 * 
 * Calculates the speed for each motor (right and left) by subtracting and
 * adding the PID value to the base speed. The constrain function ensures
 * that the speed values remain within the valid range (0-255).
 * It then drives the motors with the new speeds.
 */
void adjust_movement() { 
  // Change the speed value
  right_speed = constrain(base_right_speed - PID, 0, 255);
  left_speed = constrain(base_left_speed + PID, 0, 255);
  // Send the new speed to the run function
  run(right_speed, left_speed);
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
void calculate_error() {
  // Update sensor values
  read_sensors();

  // Check for inversion, signaling a pedestrian crossing
  check_inversion(SENSOR, CURVE_SENSORS);

  if (inversion_finished) {
    pedestrian_crossing_detected = true;
  }

  // Initialize variables for error calculation
  int pesos[5] = {-2, -1, 0, 1, 2};
  int error_sum = 0;
  int active_sensors = 0;

  // Perform a summation with the sensor values and weights
  for (int i = 0; i < 5; i++) {
    error_sum += SENSOR[i] * pesos[i];
    active_sensors += SENSOR[i];
  }

  // Determine the car's error
  if (active_sensors == TOTAL_SENSORS) {
    error = LINE_NOT_DETECTED;
  } else {
    int inactive_sensors = TOTAL_SENSORS - active_sensors;
    error = error_sum / inactive_sensors;
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
void calculate_PID() {
  // Initialize variables for calculation
  PID = 0;
  P = error;
  I = constrain(I + P, -255, 255);
  D = error - previous_error;

  // Calculate PID
  PID = (Kp * P) + (Ki * I) + (Kd * D) + OFFSET;

  // Update the previous error value
  previous_error = error;
}

/**
 * @brief Prints debugging information to the serial monitor.
 * 
 * Sends the state of the curve and line sensors, the calculated error value,
 * the PID output value, and the resulting speeds for the right and left motors
 * to the serial. Useful for calibrating and monitoring the robot's behavior.
 */
void print_serial() {
  // Print sensor values
  Serial.print(CURVE_SENSORS[0]);
  Serial.print(" | ");

  for (int i = 0; i < 5; i++) {
    Serial.print(SENSOR[i]);
    Serial.print(" | ");
  }

  Serial.print(CURVE_SENSORS[1]);
  Serial.print(" | ");

  // Print Error, PID, and speed variables
  Serial.print("\tErro: ");
  calculate_error();
  Serial.print(error);
  Serial.print(" PID: ");
  calculate_PID();
  Serial.print(PID);
  Serial.print(" Velocidade Direita: ");
  Serial.print(right_speed);
  Serial.print(" Velocidade Esquerda: ");
  Serial.println(left_speed);
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
    logFile.println("Time,Error,Challenge,RightMarkers,LeftMarkers,T_Right,T_Left,RightSpeed,LeftSpeed,sc0,s0,s1,s2,s3,s4,sc1"); 
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
    logFile.print(error);
    logFile.print(",");
    logFile.print(challenge_marker);
    logFile.print(",");
    logFile.print(right_markers);
    logFile.print(",");
    logFile.print(left_markers);
    logFile.print(",");

    logFile.print(right_marker_time);
    logFile.print(",");
    logFile.print(left_marker_time);
    logFile.print(",");

    logFile.print(right_speed);
    logFile.print(",");
    logFile.print(left_speed);

    // Adiciona valores dos 5 sensores da linha
    for (int i = 0; i < 5; i++) {
      logFile.print(",");
      logFile.print(SENSOR[i]);
    }

    // Adiciona sensores de curva
    for (int i = 0; i < 2; i++) {
      logFile.print(",");
      logFile.print(CURVE_SENSORS[i]);
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


unsigned int lost_line_counter = 0;
unsigned long time_since_line_lost = 0;
int recovery_attempts = 0;
const int RECOVERY_ATTEMPT_LIMIT = 3;
unsigned long last_recovery_time = 0;
const unsigned long ATTEMPT_RESET_TIME = 3000;

unsigned int inversion_end_counter = 0;

void loop() {  
  // verifica o estado do led
  check_led_status();

  if (!debugMode) {
    if (sprint_mode) {
      read_sensors();
      calculate_error();
      calculate_PID();
      adjust_movement();
      if (debugSD) write_sd(0);
    } else {
      read_sensors();
      int position = calculate_position(SENSOR);

      if (position != 0) {
        last_line_position = position;  // store last valid reading
      }
      // verifica se tem uma curva de 90
      int curve_exit = check_90_degree_curve(SENSOR, CURVE_SENSORS);
      if (!inversion_active){
        if (curve_exit != NO_CURVE_FOUND && (millis() - last_curve_time < CURVE_DEBOUNCE_TIME)) {
          curve_exit = NO_CURVE_FOUND; 
        }
      }

      if (pedestrian_crossing_detected) {
        // filtro de ruido
        if (count_active_sensors(SENSOR) == TOTAL_SENSORS) {
          inversion_end_counter += 1;
        }

        if (inversion_end_counter >= 3) {
          if (debugSD) write_sd(2);
          perform_pedestrian_crossing();
          pedestrian_crossing_detected = false;
        }
      }
      
      if (curve_exit == NO_CURVE_FOUND) {
        calculate_error();
        
        if (error != LINE_NOT_DETECTED) {
          // segue normalmente
          calculate_PID();
          if ((CURVE_SENSORS[0] == BLACK && CURVE_SENSORS[1] == BLACK) || inversion_active) {
            adjust_movement();
          }

          if (millis() - last_recovery_time > ATTEMPT_RESET_TIME) {
            recovery_attempts = 0;
          }

          if (debugSD) write_sd(0);
          lost_line_counter = 0; // reset if line is found
        } else if (error == LINE_NOT_DETECTED) {
          // perdeu a linha
          if (lost_line_counter == 0) {
            time_since_line_lost = millis(); // mark when it was lost
          }
          lost_line_counter++;

          // verifica se a perda já é relevante (por contagem OU tempo)
          if (lost_line_counter > LOST_LINE_TOLERANCE_LIMIT || millis() - time_since_line_lost > 1000) {
                
            stop_motors();

            // tenta recuperar a linha
            if (try_to_recover_line()) {
              lost_line_counter = 0; // recovered successfully
              recovery_attempts++;
              last_recovery_time = millis();
              
              if (recovery_attempts >= RECOVERY_ATTEMPT_LIMIT) { 
                // já tentou muitas vezes → para definitivo
                error = previous_error;
                if (debugSD) write_sd(3);
                perform_stop_area();
              }
            } else {
              error = previous_error;
              if (debugSD) write_sd(3); // Log challenge 3: Stop area
              perform_stop_area(); // couldn't recover -> stop
            }
          }
        }
      } else if (curve_exit != NO_CURVE_FOUND) {
        // evita ligar no cruzamento
        if (count_active_sensors(SENSOR) > 1) {
          if (!inversion_active) {
            right_markers = 0;
            left_markers = 0;
            
            analyze_markers();
            
            if (right_markers == 1 && left_markers == 1 ) {
              unsigned long time_delta;
              if (right_marker_time > left_marker_time) {
                time_delta = right_marker_time - left_marker_time;
              } else {
                time_delta = left_marker_time - right_marker_time;
              }

              if (time_delta >= SIMULTANEOUS_TIME_TOLERANCE) {
                curve_exit = (right_marker_time < left_marker_time) ? RIGHT_CURVE : LEFT_CURVE;
                perform_reverse_gear(curve_exit);
              } else {
                turn_90(DOUBTFUL_CURVE); // it's inverted, I don't know why
                if (debugSD) write_sd(6);
              }
            } else if (right_markers >= 1 || left_markers >= 1) {
              turn_90(curve_exit);
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
      read_sensors();
      print_serial();
    }
  }

  delay(5);
}