#include "main.h"
#include "config.hpp"
#include "config.hpp"

const double WHEEL_RADIUS = 5.08;
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
 static void unBrakeMotors(){
   left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 }
 static void brakeMotors(){//brake the base motors
   left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   left_wheel.move_velocity(0);
   left_chain.move_velocity(0);
   right_wheel.move_velocity(0);
   right_chain.move_velocity(0);
 }



 void turn(double goal){//simple gyro turn function (positive to right)
   gyro.reset();//sets gyro value to 0
   double error = goal - gyro.get_value() / 10.0;
   while(error != 0.0){//while there is error
     if(error > 0){//turns right if goal positive
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
     error = goal - gyro.get_value();
     pros::delay(2);
   }//stops movement after turn
   brakeMotors();
   unBrakeMotors();
 }



 void moveP(double distance, int multi){
   double circumference = WHEEL_RADIUS * 2.0 * M_PI;
   double goal = (distance/circumference)*360.0; //make error into degrees
   double encoderavg = (right_wheel.get_position() + right_chain.get_position() +  left_wheel.get_position()
                             + left_chain.get_position())/4;
   double error = goal - encoderavg;

   double Kp = 0.5;
   double speed;

   right_wheel.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);//sets wheels to encode in degrees
   right_chain.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   left_wheel.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   left_chain.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

   right_wheel.tare_position();//sets encoders position to 0 degrees
   right_chain.tare_position();
   left_wheel.tare_position();
   left_chain.tare_position();

   while(error != 0){
     speed = error * Kp;

     if(speed > 200){//sets max speed to +/- 200
       speed = 200;
     }else if (speed < -200){
       speed = -200;
     }

     if(multi == 1){//setting multitask
       intake.move_velocity(-200); //intake out
     } else if (multi == 2){
       intake.move_velocity(200); //intake in
     } else if (multi == 3){
       angler.move_velocity(-200);
     } else if (multi == 4){
       angler.move_velocity(200);
     }
     left_wheel.move_velocity(speed);
     left_chain.move_velocity(speed);
     right_wheel.move_velocity(speed);
     right_chain.move_velocity(speed);

     error = goal - encoderavg;//recalculate error
     pros::delay(2); //don't hog cpu
   }
   brakeMotors();
   unBrakeMotors();
 }
void redclose(){
  /**
    looking at close flag
    angle down
    punch mid flag
    angle up
    turn right 90
    straight
    intake ball
    go back onto wall
    foward a bit
    turn left 90
    punch top flag
    straight toggle low flag
    back one tile
    if park, back onto platform tile, turn right 90, straight onto platform
    if not, turn 90 degrees right, intake out and straight
  **/
}

void redfar(){
  /**
  **/

}

void blueclose(){
  /**
  **/
}

void bluefar(){
  /**
  **/
}
/**
 void autonSelector(){
   if(redside chosen autonColor == 0)
   {
     if(flag side yes? autonSide == 0){
          redclose();
     }
       else{ //far side
          redfar();
     }
   }
 else {//blue side chosen
   if(flag side yes? autonSide == 0){
      blueclose();
   }
     else{ //far side
      bluefar();
   }
 }
}**/
void tempauton(){
  moveP(60.0, 1); //move one tile while intake out
  turn(90.0);//turn 90 degrees to the right
}
void autonomous() {
  //autonSelector();
  tempauton();
}
