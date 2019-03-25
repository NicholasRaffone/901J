#include "main.h"
#include "cmath"
#include "config.hpp"
#include "auton_functions.h"
//UNITS ARE INCHES!!!!
const float WHEEL_RADIUS = 1.625;
const double CIRCUMFERENCE = 2*M_PI*WHEEL_RADIUS;
const float ENCODERTICKSPERREVOLUTION = 360.0;
const int DEFAULTSLEWRATEINCREMENT = 25;

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

void slewRateControl(pros::Motor *motor, int targetVelocity, int increment){
  int currentVelocity = motor->get_actual_velocity();
  if (targetVelocity != 0){
    if (currentVelocity != targetVelocity){
      if (targetVelocity > currentVelocity){
        currentVelocity += increment;
      } else if (targetVelocity < currentVelocity){
        currentVelocity -= increment;
      }
      if (std::abs(currentVelocity) > std::abs(targetVelocity)){
        currentVelocity = targetVelocity;
      }
    }
  } else {
    currentVelocity = targetVelocity;
  }
  motor->move_velocity(currentVelocity);
  printf("wtd %d\r\n", currentVelocity);
}

void move_PID(float targetDistance, int maxVelocity, int multiTask){

  const double degreeGoal = (targetDistance/CIRCUMFERENCE)*ENCODERTICKSPERREVOLUTION;
  bool goalMet = false;
  int targetVelocity = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.45;
  double kI = 0.0002;
  double kD = 0.02;
  double integral = 0;
  double derivative = 0;

  if (targetDistance < 0) {maxVelocity *= -1;}
  mainEncoder.reset();

  while(!goalMet){
    currentPosition = mainEncoder.get_value();
    error = degreeGoal - currentPosition;

    if (std::abs(error) < 720){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (targetVelocity > maxVelocity){
      targetVelocity = maxVelocity;
    }

    slewRateControl(&left_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&left_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);

    /**
    left_wheel.move_velocity(targetVelocity);
    left_chain.move_velocity(targetVelocity);
    right_wheel.move_velocity(targetVelocity);
    right_chain.move_velocity(targetVelocity);
    **/
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0){
      goalMet = true;
    }
    //if (std::abs(error) < 10){
      //goalMet = true;
    //}
    pros::delay(10);
  }
  brakeMotors();
}
