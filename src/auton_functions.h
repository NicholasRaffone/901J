#include "main.h"
#include "cmath"
#include "config.hpp"

extern const float WHEEL_RADIUS;
extern const double CIRCUMFERENCE;
extern const float ENCODERTICKSPERREVOLUTION;
extern const int DEFAULTSLEWRATEINCREMENT;

void brakeMotors();
void unBrakeMotors();
void slewRateControl(pros::Motor *motor, int targetVelocity, int increment);
void move_PID(float targetDistance, int maxVelocity, int multiTask);
