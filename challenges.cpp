#include "challenges.h"
#include "motors.h"
#include "constants.h"
#include "CoRA2025.h"

void check_led_status() {
  // check if the led is on
  if (led_on) {
    // check if the led has been on for more than MAX_LED_ON_TIME
    if (millis() - led_on_time >= MAX_LED_ON_TIME) {
      digitalWrite(LEDS, LOW);
      led_on = false;
    }
  }
}

/**
 * @brief Calculates the number of active sensors.
 * 
 * An active sensor is one that detects the color black (value 1).
 * 
 * @param SENSOR Array with the state of the 5 main line-following sensors.
 * @return int The total number of active sensors.
 */
int count_active_sensors(int SENSOR[]) {
  int active_sensors = 0;
  // Calculates the number of active sensors
  for(int i = 0; i < 5; i++) {
    active_sensors += SENSOR[i];
  }
  
  // Returns the number of active sensors
  return active_sensors;
}

/**
 * @brief Determines if the robot should perform a 90-degree turn.
 * 
 * Esta função analisa o estado dos sensores principais e de curva para identificar
 * um padrão de curva de 90 graus. Para evitar falsos positivos devido a ruídos
 * nos sensores, ela exige que uma curva seja detectada por um número mínimo de
 * leituras consecutivas (3, por padrão) antes de confirmar a curva e retornar sua direção.
 * 
 * @param SENSOR Array containing the states of the main sensors.
 * @param CURVE_SENSORS Array containing the states of the curve detection sensors.
 * 
 * @return int An integer representing the turn direction:
 * - `LEFT_CURVE` if a left turn is confirmed.
 * - `RIGHT_CURVE` if a right turn is confirmed.
 * - `DOUBTFUL_CURVE` if the turn direction is confirmed but uncertain.
 * - `NO_CURVE_FOUND` if no curve is detected or the detection threshold is not met.
 */

int check_90_degree_curve(int SENSOR[], int CURVE_SENSORS[]) {
  int black_sensors = count_active_sensors(SENSOR);

  // Right curve
  if (CURVE_SENSORS[0] == BLACK && black_sensors >= 2 && CURVE_SENSORS[1] == WHITE) {
    return LEFT_CURVE;
  }

  // Left curve
  if (CURVE_SENSORS[0] == WHITE && black_sensors >= 2 && CURVE_SENSORS[1] == PRETO) {
    return RIGHT_CURVE;
  }

  // Case of doubt (straight line with a strange mark)
  if (SENSOR[1] == BLACK && SENSOR[3] == BLACK && CURVE_SENSORS[0] == WHITE && CURVE_SENSORS[1] == WHITE) {
    return DOUBTFUL_CURVE;
  }

  return NO_CURVE_FOUND;
}

int calculate_position(int SENSOR[]) {
  int weights[5] = {-2, -1, 0, 1, 2};
  int weight_sum = 0, active_sum = 0;

  for (int i = 0; i < 5; i++) {
    if (SENSOR[i] == BLACK) {
      weight_sum += weights[i];
      active_sum++;
    }
  }

  if (active_sum == 0) return 0; // Reading failure
  return weight_sum / active_sum; // average value
}


/**
 * @brief Executes a 90-degree turn using the gyroscope.
 * 
 * Based on the detected turn direction, this function stops the robot for
 * stability and then performs a precise 90-degree turn.
 * 
 * @param found_curve The direction of the turn, e.g., `LEFT_CURVE` or `RIGHT_CURVE`.
 */
void turn_90(int found_curve) {
  
  if (!inversion_active) {
    unsigned long startTime = millis();
    // This timeout prevents the robot from getting stuck if it misreads the sensors.
    // (millis() - startTime < TIMEOUT_90_CURVE)
    while (count_active_sensors(SENSOR) <= 1) {
      // Move straight forward, not using line-following logic here.
      run(base_right_speed, base_left_speed);
      read_sensors(); // Keep updating sensor values to check the condition.
    }
    delay(50);
    
    stop_motors();
    delay(100);
    int position = calculate_position(SENSOR);
  
    // Proportional adjustment (e.g., 10 degrees per position)
    int adjustment = position * 0;  
  
    int final_angle = 80 + adjustment;
  
    // Stop the car for greater stability
    stop_motors();
    delay(200);
  
    error = previous_error;
    if (found_curve == LEFT_CURVE) {
      if (debugSD) write_sd(1);
      digitalWrite(LEDS, HIGH);

      //while (SENSOR[2] != WHITE) {
       // turn_right(base_right_speed, base_left_speed);
        //read_sensors();
      //}
      turn_right(base_right_speed, base_left_speed);
      turn_until_angle(final_angle); 
    } else if (found_curve == RIGHT_CURVE) {
      if (debugSD) write_sd(4);
      digitalWrite(LEDS, HIGH);
      turn_right(base_right_speed, base_left_speed);
      turn_until_angle(final_angle);
    }
  
    stop_motors();
    delay(100); 
    // MOVE A BIT TO AVOID DOUBLE DETECTION
    while (true) {
      if (debugSD) write_sd(13);
      calculate_error();
      calculate_PID();
      adjust_movement();
      if (CURVE_SENSORS[0] == BLACK && CURVE_SENSORS[1] == BLACK && SENSOR[2] == WHITE) {
        break;
      }
    }
  
    last_curve_time = millis();
    // reset marker values to avoid false positives
    right_markers = 0; left_markers = 0;
  }
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
void turn_until_angle(int target_angle = 90) {
  unsigned long previous_time = millis();
  float angle_z = 0;

  while (abs(angle_z) < target_angle) {
    mpu.update(); 
    float angular_velocity_z = mpu.getGyroZ() - gyro_bias_z;

    unsigned long current_time = millis();
    float delta_time = (current_time - previous_time) / 1000.0; 
    previous_time = current_time;

    angle_z += angular_velocity_z * delta_time;

    if (debugMode) Serial.println(angular_velocity_z);
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
int invert_sensor(int sensor_value){
  if (sensor_value == BLACK){ 
    return WHITE;
  } 
  return BLACK;
}

unsigned long tempoInversaoAtivada = 0;

bool verifica_inversao(int SENSOR[], int SENSOR_CURVA[]) {
  int ativos = calcula_sensores_ativos(SENSOR);

  // or active_sensors == 1, need to check
  if (ativos == 1 && !inversion_active && SENSOR_CURVA[0] == WHITE && SENSOR_CURVA[1] == WHITE) {
    inversion_active = true;
    inversion_activation_time = millis();
    if (debugSD) write_sd(10);
  }

  if (inversion_active && millis() - inversion_activation_time > 1000) { // only turns off after 200ms
    if (ativos >= 3 && SENSOR_CURVA[0] == BLACK && SENSOR_CURVA[1] == BLACK) {
      inversion_active = false;
      if (debugSD) write_sd(11);
      inversion_finished = true;
      return false;
    }
  }

  // If inversion mode is active, invert the sensor readings.
  if (inversion_active) {
    for (int i = 0; i < 5; i++) {
      SENSOR[i] = inverte_sensor(SENSOR[i]);
    }    
    return true; 
  }

  // Se não entrou na lógica, retorna false.
  return false;
}

void perform_pedestrian_crossing() {
  if (debugSD) write_sd(2);
  stop_motors();
  delay(PEDESTRIAN_CROSSING_TIMEOUT); 
  // needs logic that aligns with almost 100% certainty so it doesn't lose the line it will find ahead
  
  if (debugSD) write_sd(12);
  unsigned long start = millis();
  while (error == LINE_NOT_DETECTED || millis() - start < 700) {
    read_sensors();
    calculate_error();
    run_backward(150, 150);
  }
  stop_motors();
  
  int pos = calculate_position(SENSOR); // -2..2
  if (pos < -1) {
    run(-base_right_speed, base_left_speed);
  } else if (pos > 1) {
    run(base_right_speed, -base_left_speed);
  }

  while (SENSOR[1] != BLACK && SENSOR[2] != WHITE && SENSOR[3] != BLACK) {
    read_sensors();
  }
  
  stop_motors();
  delay(500);

  if (debugSD) write_sd(14);
  
  run(255, 255);
  delay(CROSSING_PERIOD_TIMEOUT);
  
  read_sensors();
  calculate_error();
  while(SENSOR[2] != WHITE) {
    run(255, 255);
    read_sensors();
    calculate_error();
  }

  while(SENSOR[1] == BLACK && SENSOR[2] == WHITE && SENSOR[3] == BLACK) {
    read_sensors();
    calculate_error();
    calculate_PID();
    adjust_movement();
  }

  if(debugSD) write_sd(15); // Log: Fim da travessia
  
  // Zera a flag para não entrar neste desafio novamente por engano
  inversion_finished = false;
} 

/**
 * @brief Determines the exit direction for a challenge based on marker counts.
 * 
 * Compares the number of markers detected on the left and right sides.
 * The robot should exit on the side with fewer markers.
 * 
 * @param left_markers The number of markers detected on the left.
 * @param right_markers The number of markers detected on the right.
 * @return int The determined exit side (`RIGHT_EXIT` or `LEFT_EXIT`).
 */
int determine_curve_exit(int left_markers, int right_markers) {
  if (right_markers < left_markers) {
    return RIGHT_EXIT;
  } else {
    return LEFT_EXIT;
  }
}


/**
 * @brief Determines the roundabout exit based on the number of detected markers.
 * 
 * Based on the total number of markers seen before entering the roundabout,
 * this function sets the desired exit number.
 * 
 * @param curve_exit The direction of the curve to enter the roundabout (currently unused).
 * @param number_of_marks The total number of markers detected.
 * @return int The desired exit number (1, 2, or 3).
 */
int determine_roundabout_exit(int curve_exit, int number_of_marks) {
  if (number_of_marks == 2) {
    desired_exit = 1; // 1st Exit
  } else if (number_of_marks == 3) {
    desired_exit = 2; // 2nd Exit
  } else if (number_of_marks >= 4) { 
    desired_exit = 3; // 3rd Exit
  }
  
  return desired_exit;
}

/**
 * @brief Navigates the roundabout challenge.
 * 
 * The robot enters the roundabout, follows the line, and counts exits until
 * it reaches the `saidaDesejada`. It then takes that exit.
 * 
 * @param curve_exit The direction to turn to enter the roundabout.
 * @param desired_exit The target exit number to take.
 */
void perform_roundabout(int curve_exit, int desired_exit) {
  if (debugSD) write_sd(8);
  
  // Flag to control if we are looking for an exit or waiting to pass one already counted
  bool waiting_for_realign = false;
  
  // Counter to add tolerance to exit detection
  static int exit_detection_counter = 0;
  const int EXIT_TOLERANCE = 2; // Requires 2 consecutive readings to confirm the exit

  // Reset the count at the beginning of the challenge
  int current_exit = 0;

  // 1. Turn 90 degrees to enter the roundabout (using the gyroscope)
  if (curve_exit == LEFT_EXIT) {
    turn_left(base_right_speed, base_left_speed);
    turn_until_angle(90);
  } else if (curve_exit == RIGHT_EXIT) {
    turn_right(base_right_speed, base_left_speed);
    turn_until_angle(90);
  }

  // 2. Navigate the roundabout until the correct exit is found
  while (current_exit < desired_exit) {
    // Main line-following logic is executed in every cycle
    calculate_error();
    adjust_movement();

    // Assuming the roundabout is to the right (counter-clockwise)
    bool exit_detected = (CURVE_SENSORS[0] == BLACK && CURVE_SENSORS[1] == WHITE);
    
    // Condition indicating the robot is back on the main curve
    bool robot_realigned = (CURVE_SENSORS[0] == BLACK && CURVE_SENSORS[1] == BLACK);

    // Main counting logic
    if (!waiting_for_realign) {
      // search for the exit
      if (exit_detected) {
        exit_detection_counter++; // Increment counter if a possible exit is seen
      } else {
        exit_detection_counter = 0; // Reset if the condition fails
      }

      // If the exit has been seen for enough readings, count it and activate the wait flag
      if (exit_detection_counter >= EXIT_TOLERANCE) {
        current_exit++;
        waiting_for_realign = true; // Activate the flag to stop searching
        exit_detection_counter = 0; // Reset the counter for the next search
      }
    } else {
      // If the flag is active, we wait for the robot to realign to deactivate it
      if (robot_realigned) {
        waiting_for_realign = false; // Deactivate the flag, allowing the search for the next exit
      }
    }
  }

  // exit the roundabout, the robot will already be aligned with the exit, just turn 90 degrees
  if (curve_exit == LEFT_EXIT) {
      turn_right(base_right_speed, base_left_speed);
      turn_until_angle(90);
  } else {
      turn_left(base_right_speed, base_left_speed);
      turn_until_angle(90);
  }
}

/**
 * @brief Executes the reverse gear challenge.
 * 
 * The robot stops, moves backward for a fixed duration, stops again,
 * and then turns 90 degrees to the specified side.
 * 
 * @param curve_side The side to turn towards after reversing (`RIGHT_EXIT` or `LEFT_EXIT`).
 */
void perform_reverse_gear(int curve_side) {
  if (debugSD) write_sd(7);
  stop_motors();
  delay(1000);

  run_backward(255, 255); 
  delay(1200);

  // 3. Stop again
  stop_motors();
  delay(500);

  run(base_right_speed, base_left_speed);
  delay(200);

  turn_90(curve_side);

  stop_motors();
  delay(500);
}

bool try_to_recover_line() {
  unsigned long time_lost = millis();
  int valid_readings = 0;

  if (debugSD) write_sd(5); // perda de linha

  while (millis() - time_lost < TIME_WITHOUT_LINE) {
    run_backward(base_right_speed, base_left_speed);
    delay(50);
    read_sensors();

    // criterion: central sensor detects line
    if (SENSOR[1] == BLACK || SENSOR[2] == BLACK || SENSOR[3] == BLACK) {
      valid_readings++;
      if (valid_readings >= 3) { // requires 3 consecutive readings
        stop_motors();
        return true;
      }
    } else {
      valid_readings = 0; // lost it again
    }
  }

  //stop_motors();
  return false;
}

/**
 * @brief Counts a marker (black square) using rising edge detection.
 * 
 * Increments the marker count only on the transition from WHITE to BLACK.
 * A control flag (`jaContou`) is used to ensure that the same marker
 * is not counted multiple times while the sensor remains over it.
 * 
 * @param sensor_state The current state of the sensor (BLACK or WHITE).
 * @param current_count The current value of the marker counter.
 * @param already_counted Reference to a flag indicating if the current marker has already been counted.
 * @return int The updated marker count.
 */
int white_counter = 0;
const int MARKER_TOLERANCE = 3; 
int count_marker(int sensor_state, int current_count, bool &already_counted, int &white_counter, unsigned long &marker_time) {
  // avoid detecting intersections
  if (count_active_sensors(SENSOR) <= 1) return current_count;
  // avoid false positive bug
  
  if (right_markers >= 1 && left_markers >= 1) return current_count;

  if (sensor_state == WHITE) {
    white_counter++;
    // Only count if the threshold is reached and it hasn't been counted yet
    if (white_counter >= MARKER_TOLERANCE && !already_counted) {
      already_counted = true;
      marker_time = millis(); 
      return current_count + 1;
    }
  } else {
    // Reset when leaving the white area
    white_counter = 0;
    already_counted = false;
  }
  return current_count;
}


void analyze_markers() {
  if (!inversion_active) {
    unsigned long last_detection_time = millis();
    already_counted_left = false;
    already_counted_right = false;
    
    // If there is a curve, store the number of markers
    while((millis() - last_detection_time < MARKER_TIMEOUT)) {
      // Update sensor values
      read_sensors();
      
      // conta as marcacoes
      static int white_counter_left = 0;
      static int white_counter_right = 0;

      int markers_before_left = left_markers;
      int markers_before_right = right_markers;

      left_markers = count_marker(CURVE_SENSORS[0], left_markers, already_counted_left, white_counter_left, left_marker_time);
      right_markers  = count_marker(CURVE_SENSORS[1], right_markers, already_counted_right, white_counter_right, right_marker_time);

      if (left_markers > markers_before_left || right_markers > markers_before_right) {
        last_detection_time = millis();
      }

      if (right_markers >= 1 && left_markers >= 1) {
        right_markers = 1;
        left_markers = 1;
      }

      // Ensure the robot stays on the line
      calculate_error();
      calculate_PID();
      adjust_movement();
      if (debugSD) write_sd(9);
    }
  }
}

void area_de_parada() {
  run(velocidadeBaseDireita - 5, velocidadeBaseEsquerda);
  delay(1000);
  stop_motors();
  digitalWrite(LEDS, HIGH);
  led_on_time = millis();
  led_on = true;
  if (debugMode) Serial.println("Área de parada detectada. Robô parado.");
  if (debugSD) write_sd(3); // Log challenge 3: Stop area
  while(true);
}

void test_motors() {
    //run(base_right_speed, base_left_speed);
  //delay(3000);
  //stop_motors();
  //delay(1000);
  //run_backward(base_right_speed, base_left_speed);
  //delay(3000);
  //stop_motors();
  //delay(1000);
  //turn_90(RIGHT_CURVE);
  //stop_motors();
 // delay(1000);
  //turn_90(LEFT_CURVE);
 // stop_motors();
  //delay(1000);

  stop_motors();
  delay(1000);
  run(255, 255);
  delay(TIMEOUT_PERIODO_FAIXA);
  stop_motors();
  delay(1000);
}