#include "constants.h"

// Pinos dos Sensores de Linha
const int left_curve_sensor_pin = 2;
const int left_sensor_pin = 4;
const int left_central_sensor_pin = 8;
const int central_sensor_pin = 17; //A3
const int right_central_sensor_pin = 14; //A0
const int right_sensor_pin = 15; //A1
const int right_curve_sensor_pin = 16; //A2

// Variáveis de Estado dos Sensores
int SENSOR[5];
int CURVE_SENSORS[2];

// Giroscópio
const int chipSelect = 10;
float gyro_bias_z = 0.0;

// --- Motores ---
const int MOTOR_OFFSET = 5;
const int base_speed = (!sprint_mode) ? 210 : 255;
const int base_right_speed = base_speed - MOTOR_OFFSET;
const int base_left_speed = base_speed;
int right_speed = 0;
int left_speed = 0;


// --- Timeouts ---
const int PEDESTRIAN_CROSSING_TIMEOUT = 5100;
const int MARKER_TIMEOUT = 400; // 550
const int MAX_LED_ON_TIME = 1500; // 3 seconds
const int TIME_WITHOUT_LINE = 200;
const int CROSSING_PERIOD_TIMEOUT = 2000;

// --- Tolerâncias ---
const int LOST_LINE_TOLERANCE_LIMIT = 50;

// --- PID ---
// Constantes para o cálculo do PID
const float Kp = (!sprint_mode) ? 180 : 150; 
const float Ki = (!sprint_mode) ? 0.015 : 0;
const float Kd = (!sprint_mode) ? 150 : 0;

float error = 0;
float previous_error = 0;
float I = 0, P = 0, D = 0, PID = 0;

// --- LEDs ---
const int LEDS = 7;
int led_on_time = 0;
bool led_on = false;


// --- Desafios ---
int roundabout_exit = -1;
bool pedestrian_crossing_detected = false;
int desired_exit = 0;
bool inversion_active = false;
int right_markers = 0, left_markers = 0;
bool already_counted_left = false, already_counted_right = false;
bool inversion_finished = false;
int last_line_position = 0;

const unsigned long CURVE_DEBOUNCE_TIME = 1000;
unsigned long last_curve_time = 0;

unsigned long right_marker_time = 0;
unsigned long left_marker_time = 0;
const int SIMULTANEOUS_TIME_TOLERANCE = 300;
// --- Debug ---
bool debugMode = false;
bool debugMotor = false;
bool debugSD = true;
bool sprint_mode = false;