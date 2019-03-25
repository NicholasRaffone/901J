#include "main.h"
#include "cmath"
#include "config.hpp"

extern const float WHEEL_RADIUS;
extern const double CIRCUMFERENCE;
extern const float encoderTicksPerRevolution;

void brakeMotors();
void unBrakeMotors();
void move_PID(float targetDistance, int maxVelocity, int multiTask);
void BRUH();
