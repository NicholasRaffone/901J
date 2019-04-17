#include "main.h"

extern const int LEFT_WHEEL_PORT; //17
extern const int LEFT_CHAIN_PORT; //18
extern const int RIGHT_CHAIN_PORT; //14
extern const int RIGHT_WHEEL_PORT; //15
extern const int INTAKE_MOTOR_PORT; //10
extern const int PUNCHER_MOTOR_PORT; //20
extern const int ANGLER_MOTOR_PORT; //11
extern const char GYRO_PORT; //C
extern const char GYRO2_PORT; //D
extern const char ENCODER_TOP_PORT; //A
extern const char ENCODER_BOT_PORT; //B
extern const int ARM_MOTOR_PORT;
extern const char ULTRASONIC_OUTPUT;
extern const char ULTRASONIC_INPUT;
extern const char LINESENSOR_INPUT;

extern pros::Motor left_wheel;
extern pros::Motor right_wheel;
extern pros::Motor left_chain;
extern pros::Motor right_chain;
extern pros::Controller master;
extern pros::Motor intake;
extern pros::Motor puncher;
extern pros::Motor arm;
extern pros::Motor angler;
extern pros::ADIGyro gyro;
extern pros::ADIGyro gyro2;
extern pros::ADIEncoder mainEncoder;
extern pros::ADIUltrasonic ultrasonic;
extern pros::ADILineSensor ballSensor;

extern int calibrate_min;
extern int calibrate_max;
extern bool blueSide; //false = red, true = blue
extern bool farSide; //false = close, true = far
extern bool park;
