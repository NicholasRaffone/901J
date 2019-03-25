#include "main.h"
#include "config.hpp"
#include "config.hpp"
#include "auton_functions.h"

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


 void angle(bool up){
   if(up){
     angler.move_relative(-75,1000);
   }else{
     angler.move_relative(75,1000);
   }
   angler.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   angler.move_velocity(0);
 }

 void intakeball(){
   intake.move_relative(500,1000);
   angler.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   angler.move_velocity(0);
 }

/*
 void turnP(double goal){//simple gyro turn function (positive to right)
   gyro.reset();//sets gyro value to 0
   double error = goal - gyro.get_value() / 10.0;
   double Kp = 0.6;
   int velocity;
   if (goal < 0 && velocity > 0) { velocity *= -1.0; }
   while(error != 0.0){//while there is error
       velocity = error * Kp;
       left_wheel.move_velocity(velocity);
       left_chain.move_velocity(velocity);
       right_wheel.move_velocity(-1*velocity);
       right_chain.move_velocity(-1*velocity);

     error = goal - gyro.get_value()/ 10.0;
     pros::delay(2);
   }//stops movement after turn
   brakeMotors();
   unBrakeMotors();
 }

 }*/
void redclose_nopark(){
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
    turn 90 degrees right, intake out and straight
  **/
}

void redclose_park(){
  /**
  **/
}

void redfar_nopark(){
  /**

  **/

}

void redfar_park(){
  /**

  **/

}

void blueclose_nopark(){
  /**
  **/
}

void blueclose_park(){
  /**
  **/
}

void bluefar_nopark(){
  /**
  **/
}

void bluefar_park(){
  /**
  **/
}

void autonSelector(){
   if(blueSide == false)//if red
   {
     if(farSide == false){
       if(park){
          redclose_park();
        }
        else{
          redclose_nopark();
        }
     }
       else{ //far side
         if(park){
            redfar_park();
          }
          else{
            redfar_nopark();
          }
     }
   }
 else {//blue side chosen
   if(farSide == false){
     if(park){
        blueclose_park();
      }
      else{
        blueclose_nopark();
      }
   }
     else{ //far side
       if(park){
          bluefar_park();
        }
        else{
          bluefar_nopark();
        }
   }
 }
}

void autonomous() {
  autonSelector();
  //tempauton();
}
