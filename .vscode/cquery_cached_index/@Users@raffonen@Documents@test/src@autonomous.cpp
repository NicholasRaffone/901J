#include "main.h"
#include "config.hpp"

const double WHEEL_RADIUS = 5.08;;
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 void unBrakeMotors(){
   left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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

 void turn(float goal){//simple gyro turn function (positive to right)
   gyro.reset();//sets gyro value to 0
   while(goal - gyro.get_value() /10.0 != 0.0){//while there is error
     if(goal - gyro.get_value() /10.0 > 0){//turns right if goal positive
       left_wheel.move_velocity(50);
       left_chain.move_velocity(50);
       right_wheel.move_velocity(-50);
       right_chain.move_velocity(-50);
     }
     else{//left if not positive
       left_wheel.move_velocity(-50);
       left_chain.move_velocity(-50);
       right_wheel.move_velocity(50);
       right_chain.move_velocity(50);
     }
     pros::delay(2);
   }//stops movement after turn
   brakeMotors();
   unBrakeMotors();
 }



void autonomous() {
  turn(90.0);
}

void moveP(double distance){
  double circumference = WHEEL_RADIUS * 2.0 * M_PI;
  double goal = (distance/circumference)*360.0; //make error into degrees
  double encoderavg = (right_wheel.get_position() + right_chain.get_position() +  left_wheel.get_position()
                            + left_chain.get_position())/4;
  double error = goal - encoderavg;

  double Kp = 0.5;

  right_wheel.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);//sets wheels to encode in degrees
  right_chain.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  left_wheel.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  left_chain.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  right_wheel.tare_position();//sets encoders position to 0 degrees
  right_chain.tare_position();
  left_wheel.tare_position();
  left_chain.tare_position();

  while(error != 0){
    double speed = error * Kp*50;
    left_wheel.move_velocity(speed);
    left_chain.move_velocity(speed);
    right_wheel.move_velocity(speed);
    right_chain.move_velocity(speed);
    error = distance - encoderavg;
  }
  brakeMotors();
  unBrakeMotors();
}
