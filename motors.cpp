#include "motors.h"

// Define motor pins
const int MOTOR_LEFT_CLKWISE = 3;
const int MOTOR_LEFT_ANTI = 5;
const int MOTOR_RIGHT_CLKWISE = 6;
const int MOTOR_RIGHT_ANTI=9;

#define DELAY_TO_START 2000

void setup_motor() {
  pinMode(MOTOR_LEFT_CLKWISE, OUTPUT);
  pinMode(MOTOR_LEFT_ANTI, OUTPUT);
  pinMode(MOTOR_RIGHT_CLKWISE, OUTPUT);
  pinMode(MOTOR_RIGHT_ANTI, OUTPUT);
  // ensure the car starts off on 2 sec
  stop_motors();
  delay(DELAY_TO_START);
}

/**
 * @brief Sets the PWM values for all motor control pins.
 *
 * @param leftCw PWM value for left motor clockwise pin.
 * @param leftCcw PWM value for left motor counterclockwise pin.
 * @param rightCw PWM value for right motor clockwise pin.
 * @param rightCcw PWM value for right motor counterclockwise pin.
 */
void set_state_motor(int leftCw, int leftCcw, int rightCw, int rightCcw) {
  analogWrite(MOTOR_LEFT_CLKWISE, leftCw);
  analogWrite(MOTOR_LEFT_ANTI, leftCcw);
  analogWrite(MOTOR_RIGHT_CLKWISE, rightCw);
  analogWrite(MOTOR_RIGHT_ANTI, rightCcw);
}

/**
 * @brief Stops all motors.
 */
void stop_motors() {
  set_state_motor(0, 0, 0, 0);
}

/**
 * @brief Moves the vehicle forward with specified speeds.
 *
 * @param velocityRight Speed for right motor (0 to 255).
 * @param velocityLeft Speed for left motor (0 to 255).
 */
void run(int velocityRight, int velocityLeft) {
  set_state_motor(velocityLeft, 0, velocityRight, 0);
}

/**
 * @brief Moves the vehicle backward with specified speeds.
 *
 * @param velocityRight Speed for right motor (0 to 255).
 * @param velocityLeft Speed for left motor (0 to 255).
 */
void run_backward(int velocityRight, int velocityLeft) {
  set_state_motor(0, velocityLeft, 0, velocityRight);
}

/**
 * @brief Turns the vehicle to the right.
 *
 * @param velocityRight Speed for right motor (0 to 255).
 * @param velocityLeft Speed for left motor (0 to 255).
 */
void turn_right(int velocityRight, int velocityLeft) {
  set_state_motor(velocityLeft, 0, 0, velocityRight);
}

/**
 * @brief Turns the vehicle to the left.
 *
 * @param velocityRight Speed for right motor (0 to 255).
 * @param velocityLeft Speed for left motor (0 to 255).
 */
void turn_left(int velocityRight, int velocityLeft) {
  set_state_motor(0, velocityLeft, velocityRight, 0);
}