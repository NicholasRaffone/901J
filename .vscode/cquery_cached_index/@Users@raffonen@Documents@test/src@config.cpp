#include "config.hpp"
const int LEFT_WHEEL_PORT = 17; //17
const int LEFT_CHAIN_PORT = 18; //18
const int RIGHT_CHAIN_PORT = 14; //14
const int RIGHT_WHEEL_PORT = 15; //15
const int INTAKE_MOTOR_PORT = 9; //9
const int PUNCHER_MOTOR_PORT = 20; //20
const int ANGLER_MOTOR_PORT = 11; //11
const char GYRO_PORT = 'B';


pros::Motor left_wheel (LEFT_WHEEL_PORT,pros::E_MOTOR_GEARSET_18, false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_wheel (RIGHT_WHEEL_PORT, pros::E_MOTOR_GEARSET_18, true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_chain (LEFT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, false,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_chain (RIGHT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Controller master (CONTROLLER_MASTER);
pros::Motor intake(INTAKE_MOTOR_PORT,pros::E_MOTOR_GEARSET_18, true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor puncher(PUNCHER_MOTOR_PORT,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor angler(ANGLER_MOTOR_PORT, pros::E_MOTOR_GEARSET_18, true,pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIGyro gyro(GYRO_PORT);
