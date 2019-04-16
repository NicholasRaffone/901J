#include "main.h"
#include "cmath"
#include "config.hpp"
#include "auton_functions.h"
//UNITS ARE INCHES!!!!
const float ENCODER_WHEEL_RADIUS = 1.625;
const double WHEEL_RADIUS = 2.0;
const double CIRCUMFERENCE = 2*M_PI*WHEEL_RADIUS;
const double ENCODER_CIRCUMFERENCE = 2*M_PI*ENCODER_WHEEL_RADIUS;
const float ENCODERTICKSPERREVOLUTION = 360.0;
const int DEFAULTSLEWRATEINCREMENT = 15;
const int ARMGEARRATIO = 5;

void arm_stack_task(void* param){
  arm_PID(90,150);
  pros::delay(1000);
  arm.tare_position();
  arm_PID(-160,150);

}

void move_puncher(int target){
  puncher.tare_position();
  bool override = false;
  while (!(puncher.get_position() > target) && !override) {
  puncher.move_voltage(12000);
   pros::delay(5);
   if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0){
     override = true;
   }
 }
 puncher.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 puncher.move_velocity(0);
}
void setpuncher(){
  move_puncher(155);
}
void shootpuncher(){
  move_puncher(160);
  pros::delay(100);
  move_puncher(155);
}

void doublePunch(){
  move_puncher(160);
  angler.move_velocity(-170);
  intake.move_velocity(250);
  move_puncher(155);
  pros::delay(100);
  shootSensor();
}

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
  int currentVelocity = motor->get_target_velocity();
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
}


void park_PID(float targetDistance, int maxVelocity, int multiTask){ //BACK WHEELS

  const double degreeGoal = (targetDistance/CIRCUMFERENCE)*ENCODERTICKSPERREVOLUTION/1.2;
  bool goalMet = false; bool oneTime = true;
  int targetVelocity = 0;
  double motorError = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.35;
  double kI = 0.0003;
  double kD = 0.000;
  double motorkP = 0.0;
  double integral = 0;
  double derivative = 0;

  if (targetDistance < 0) {maxVelocity *= -1;}
  right_wheel.tare_position();
  left_wheel.tare_position();

  if(multiTask == 1){//setting multitask
    intake.move_velocity(-200); //intake out
  } else if (multiTask == 2){
    intake.move_voltage(12000); //intake in
  } else if (multiTask == 3){
    angler.move_velocity(-200);
  } else if (multiTask == 4){
    angler.move_velocity(200);
  }

  while(!goalMet){



    motorError = right_wheel.get_position() - left_wheel.get_position();
    currentPosition = (right_wheel.get_position() + left_wheel.get_position())/2.0;
    error = degreeGoal - currentPosition;


    if (std::abs(error) < 600){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (std::abs(targetVelocity) > std::abs(maxVelocity)){
      targetVelocity = maxVelocity;
    }

    slewRateControl(&left_wheel, targetVelocity+(motorError*motorkP), DEFAULTSLEWRATEINCREMENT/2);
    slewRateControl(&left_chain, targetVelocity+(motorError*motorkP), DEFAULTSLEWRATEINCREMENT/2);
    slewRateControl(&right_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT/2);
    slewRateControl(&right_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT/2);

    if (std::abs(error) < 10){
      goalMet = true;
    }


    if (multiTask == 5 && error < 300 && oneTime){
      std::string text("arm");
      pros::Task armStacker(arm_stack_task,&text);
      oneTime = false;
    }

    pros::delay(10);
  }
  if(multiTask == 1 || multiTask == 2){//setting multitask
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.move_velocity(0); //intake out
  } else if (multiTask == 3 || multiTask == 4){
    angler.move_velocity(0);
  }
  brakeMotors();
}

void move_align(float targetDistance, int velocity){
   const double degreeGoal = (targetDistance/CIRCUMFERENCE)*ENCODERTICKSPERREVOLUTION/1.25;
   left_wheel.tare_position();
   if (targetDistance < 0){
     velocity *= -1;
   }
   left_wheel.move_velocity(velocity);
   left_chain.move_velocity(velocity);
   right_wheel.move_velocity(velocity);
   right_chain.move_velocity(velocity);

  while (std::abs(left_wheel.get_position()) < degreeGoal) {
    pros::delay(5);
  }
}

void turn_PID(float targetDegree, int maxVelocity){

  const double degreeGoal = targetDegree*10;
  bool goalMet = false;
  int targetVelocity = 0;
  int leftTarget = 0;
  int rightTarget = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.18;
  double kI = 0.0004;
  double kD = 0.00;
  double integral = 0;
  double derivative = 0;
  if(targetDegree<0){maxVelocity *= -1;}
  gyro.reset();
  gyro2.reset();


  while(!goalMet){
    currentPosition = (gyro.get_value()+gyro2.get_value())/2;
    error = degreeGoal - currentPosition;
    printf("%f\r\n",currentPosition);
    if (std::abs(error) < 1000){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (std::abs(targetVelocity) > std::abs(maxVelocity)){
      targetVelocity = maxVelocity;
    }


      leftTarget = targetVelocity;
      rightTarget = -1*targetVelocity;


    slewRateControl(&left_wheel, leftTarget, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&left_chain, leftTarget, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_wheel, rightTarget, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_chain, rightTarget, DEFAULTSLEWRATEINCREMENT);

    if (std::abs(error) < 10){
      goalMet = true;
    }

    pros::delay(10);
  }
  brakeMotors();
}
void arc_turn_PID(float targetDegree, int maxVelocity){

  const double degreeGoal = targetDegree*10;
  bool goalMet = false;
  int targetVelocity = 0;
  int leftTarget = 0;
  int rightTarget = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.18;
  double kI = 0.0003;
  double kD = 0.00;
  double integral = 0;
  double derivative = 0;
  if(targetDegree<0){maxVelocity *= -1;}
  gyro.reset();
  gyro2.reset();


  while(!goalMet){
    currentPosition = (gyro.get_value()+gyro2.get_value())/2;
    error = degreeGoal - currentPosition;
    printf("%f\r\n",currentPosition);
    if (std::abs(error) < 1000){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (std::abs(targetVelocity) > std::abs(maxVelocity)){
      targetVelocity = maxVelocity;
    }


      leftTarget = targetVelocity;
      rightTarget = -1*targetVelocity;
    if (targetVelocity > 0){
    slewRateControl(&left_wheel, leftTarget, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&left_chain, leftTarget, DEFAULTSLEWRATEINCREMENT);
  } else if (targetVelocity <= 0){
    slewRateControl(&right_wheel,rightTarget,DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_chain,rightTarget,DEFAULTSLEWRATEINCREMENT);
  }

    if (std::abs(error) < 10){
      goalMet = true;
    }

    pros::delay(10);
  }
  brakeMotors();
}

void arm_PID(float targetDegree, int maxVelocity){
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  const double degreeGoal = (targetDegree*ARMGEARRATIO);
  bool goalMet = false;
  bool limitStart = false;
  int targetVelocity = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.7;
  double kI = 0.001;
  double kD = 0.03;
  double integral = 0;
  double derivative = 0;

  if (targetDegree < 0) {maxVelocity *= -1;}

  arm.tare_position();


  while(!goalMet){
    currentPosition = arm.get_position();
    error = degreeGoal - currentPosition;

    if (std::abs(error) < 100){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (targetVelocity > maxVelocity){
      targetVelocity = maxVelocity;
    }

    slewRateControl(&arm, targetVelocity, DEFAULTSLEWRATEINCREMENT);

    if (std::abs(error) < 1){
      goalMet = true;
    }

    pros::delay(10);
  }
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  arm.move_velocity(0);
}


void move_ultrasonic(float targetDistance, int maxVelocity, int multiTask){

  const double degreeGoal = targetDistance*10;
  bool goalMet = false;
  int targetVelocity = 0;
  double currentPosition = 0.0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.8;
  double kI = 0.0006;
  double kD = 0.01;
  double integral = 0;
  double derivative = 0;




  if(multiTask == 1){//setting multitask
    intake.move_velocity(-200); //intake out
  } else if (multiTask == 2){
    intake.move_velocity(200); //intake in
  } else if (multiTask == 3){
    angler.move_velocity(-200);
  } else if (multiTask == 4){
    angler.move_velocity(200);
  }
  while(!goalMet){
    currentPosition = (ultrasonic.get_value()/2.54);
    error = currentPosition - degreeGoal;

    if (std::abs(error) < degreeGoal*10){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (targetVelocity > maxVelocity){
      targetVelocity = maxVelocity;
    }
    printf("%f\r\n",(ultrasonic.get_value()/2.54));

    slewRateControl(&left_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&left_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);

    if (std::abs(error) < 5){
      goalMet = true;
    }
    pros::delay(10);
  }
  brakeMotors();
}

void shootSensor(){

  int minValue = 900;
  int maxValue = 2000;
  int threshold = (minValue + maxValue) / 2;
  int iterate = 0;
  intake.move_velocity(200);
  while (ballSensor.get_value() > threshold && iterate < 100){
    printf("%d\r\n",ballSensor.get_value());
    pros::delay(10);
    iterate++;
  }
  shootpuncher();
  intake.move_velocity(0);
}
