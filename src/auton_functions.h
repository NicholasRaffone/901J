#include "main.h"
#include "cmath"
#include "config.hpp"

extern const float ENCODER_WHEEL_RADIUS;
extern const double ENCODER_CIRCUMFERENCE;
extern const double WHEEL_RADIUS;
extern const double CIRCUMFERENCE;
extern const float ENCODERTICKSPERREVOLUTION;
extern const int DEFAULTSLEWRATEINCREMENT;
extern const int ARMGEARRATIO;

void puncherDelay(int target);
void setpuncher();
void shootpuncher();
void doublePunch();
void brakeMotors();
void unBrakeMotors();
void slewRateControl(pros::Motor *motor, int targetVelocity, int increment);
void move_PID(float targetDistance, int maxVelocity, int multiTask);
void park_PID(float targetDistance, int maxVelocity, int multiTask);
void move_align(float targetDistance, int maxVelocity);
void turn_PID(float targetDegree, int maxVelocity);
void arm_PID(float targetDegree, int maxVelocity);
void move_ultrasonic(float targetDistance, int maxVelocity, int multiTask); //targetDistance = how far away from wall or object
void arm_stack_task(void* param);
void shootSensor();
