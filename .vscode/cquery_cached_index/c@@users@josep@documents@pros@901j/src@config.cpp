#include "config.hpp"
const int LEFT_WHEEL_PORT = 4; //17
const int LEFT_CHAIN_PORT = 3; //18
const int RIGHT_CHAIN_PORT = 1; //14
const int RIGHT_WHEEL_PORT = 2; //15
const int INTAKE_MOTOR_PORT = 6; //9
const int PUNCHER_MOTOR_PORT = 12; //20
const int ANGLER_MOTOR_PORT = 13; //11
const int ARM_MOTOR_PORT = 7;
const char GYRO_PORT = 'C';
const char GYRO2_PORT = 'D';
const char ENCODER_TOP_PORT = 'A';
const char ENCODER_BOT_PORT = 'B';
const char LIMIT_SWITCH_PORT = 'F';
const char ULTRASONIC_OUTPUT = 'G';
const char ULTRASONIC_INPUT = 'H';

pros::Motor left_wheel (LEFT_WHEEL_PORT,pros::E_MOTOR_GEARSET_18, false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_wheel (RIGHT_WHEEL_PORT, pros::E_MOTOR_GEARSET_18, true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_chain (LEFT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_chain (RIGHT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Controller master (CONTROLLER_MASTER);
pros::Motor arm(ARM_MOTOR_PORT,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake(INTAKE_MOTOR_PORT,pros::E_MOTOR_GEARSET_18, false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor puncher(PUNCHER_MOTOR_PORT,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor angler(ANGLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, false,pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIGyro gyro(GYRO_PORT);
pros::ADIGyro gyro2(GYRO2_PORT);
pros::ADIEncoder mainEncoder(ENCODER_TOP_PORT,ENCODER_BOT_PORT,true);
pros::ADIDigitalIn armLimitSwitch(LIMIT_SWITCH_PORT);
pros::ADIUltrasonic ultrasonic(ULTRASONIC_OUTPUT,ULTRASONIC_INPUT);

bool blueSide = false; // 1
bool farSide = false; // 2
bool park = true; // 4
