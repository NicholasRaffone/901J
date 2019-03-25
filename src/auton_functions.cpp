#include "main.h"
#include "cmath"
#include "config.hpp"
#include "auton_functions.h"
//UNITS ARE INCHES!!!!
const float WHEEL_RADIUS = 1.625;
const double CIRCUMFERENCE = 2*M_PI*WHEEL_RADIUS;
const float encoderTicksPerRevolution = 360.0;

void brakeMotors(){//brake the base motors
  left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_wheel.move_velocity(0);
  left_chain.move_velocity(0);
  right_wheel.move_velocity(0);
  right_chain.move_velocity(0);
}
void unBrakeMotors(){
  left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}


void move_PID(float targetDistance, int maxVelocity, int multiTask){

  const double degreeGoal = (targetDistance/CIRCUMFERENCE)*encoderTicksPerRevolution;
  bool goalMet = false;
  int targetVelocity = 0;
  double currentPosition = 0;
  double error = 0;
  double kP = 0.3;

  if (targetDistance < 0) {maxVelocity *= -1;}
  mainEncoder.reset();

  while(!goalMet){
    currentPosition = mainEncoder.get_value();
    error = degreeGoal - currentPosition;
    if (std::abs(error) > maxVelocity){
      targetVelocity = maxVelocity;
    } else {
      targetVelocity = kP*error;
    }
    left_wheel.move_velocity(targetVelocity);
    left_chain.move_velocity(targetVelocity);
    right_wheel.move_velocity(targetVelocity);
    right_chain.move_velocity(targetVelocity);

    if (std::abs(error) < 10){
      goalMet = true;
    }
    pros::delay(10);
  }
  brakeMotors();
}
void BRUH(){
  left_wheel.move_velocity(50);
  left_chain.move_velocity(50);
  right_wheel.move_velocity(50);
  right_chain.move_velocity(50);
  pros::delay(1000);
  brakeMotors();

}
