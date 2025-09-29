#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <MPU6050_tockn.h>

//  Definições Globais
#define WHITE 0
#define BLACK 1

//  Configurações do PID e Linha 
#define OFFSET 0
#define LINE_NOT_DETECTED -5
#define TOTAL_SENSORS 5

//  Configurações de Desafios 
#define NO_CURVE_FOUND 0
#define LEFT_CURVE 1
#define RIGHT_CURVE 2
//#define DOUBTFUL_CURVE RIGHT_CURVE // change on the day
#define DOUBTFUL_CURVE RIGHT_CURVE // it's inverted
#define ANGLE_CURVE 120

#define DETECTION_BY_SQUARE 1
#define LEFT_EXIT 0
#define RIGHT_EXIT 1

//  Pinos dos Sensores 
extern const int left_curve_sensor_pin;
extern const int left_sensor_pin;
extern const int left_central_sensor_pin;
extern const int central_sensor_pin;
extern const int right_central_sensor_pin;
extern const int right_sensor_pin;
extern const int right_curve_sensor_pin;

//  Pinos dos LEDs
extern const int LEDS;

//  Variáveis de Estado dos Sensores 
extern int SENSOR[5];
extern int CURVE_SENSORS[2];
extern const int LOST_LINE_TOLERANCE_LIMIT;
extern const int chipSelect;

//  Configurações de Velocidade e Motores 
extern const int base_right_speed;
extern const int base_left_speed;
extern int right_speed;
extern int left_speed;

//  Variáveis do PID 
extern const float Kp, Ki, Kd;
extern float error;
extern float previous_error;
extern float I, P, D, PID;

//  Variáveis de Estado dos Desafios 
extern int roundabout_exit;
extern bool pedestrian_crossing_detected;
extern int desired_exit;
extern bool debugMode;
extern bool debugMotor;
extern bool debugSD;


//  Variáveis do Giroscópio 
extern MPU6050 mpu;
extern float gyro_bias_z;


extern int led_on_time;
extern const int MAX_LED_ON_TIME;
extern bool led_on;
extern bool inversion_active;
extern const int PEDESTRIAN_CROSSING_TIMEOUT;
extern const int MARKER_TIMEOUT;
extern const int TIME_WITHOUT_LINE;

extern const unsigned long CURVE_DEBOUNCE_TIME;
extern int right_markers, left_markers;

extern bool already_counted_left, already_counted_right;

extern bool inversion_finished;
extern const int CROSSING_PERIOD_TIMEOUT;
extern int last_line_position;

extern unsigned long last_curve_time;
extern bool sprint_mode;

extern unsigned long right_marker_time;
extern unsigned long left_marker_time;
extern const int SIMULTANEOUS_TIME_TOLERANCE;
#endif // CONSTANTS_H