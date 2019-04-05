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


 void intakeball(){
   intake.move_relative(500,1000);
   intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   intake.move_velocity(0);
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
       left_wheel.park_velocity(velocity);
       left_chain.park_velocity(velocity);
       right_wheel.park_velocity(-1*velocity);
       right_chain.park_velocity(-1*velocity);

     error = goal - gyro.get_value()/ 10.0;
     pros::delay(2);
   }//stops parkment after turn
   brakeMotors();
   unBrakeMotors();
 }

 }*/
void redclose_nopark(){
  setpuncher();
  park_PID(29, 150, 2);
  park_PID(-3, 50, 2);
  pros::delay(500);
  intakeball();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-32, 150, 2);
  park_PID(4, 100, 2);
  turn_PID(-90, 65);
  doublePunch();
  park_PID(36, 150, 1);
}

void redclose_park(){
  setpuncher();
  park_PID(60, 150, 2);
  park_PID(-3, 50, 2);
  intakeball();
  turn_PID(-90, 100);
  intake.move_velocity(-50);
  doublePunch();
  park_PID(-30, 150, 1);
  turn_PID(90, 100);
  park_PID(45, 200, 1);
  pros::delay(100);
  park_PID(45, 200, 1);
  brakeMotors();
  unBrakeMotors();
}

void redfar_nopark(){

  park_PID(60, 150, 1);
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
  bool list[3] = {blueSide, farSide, park}; //1, 2, 4
  int list2[3] = {1,2,4};
  int total = 0;
  for(int i = 0; i < 2; i++){
    if(list[i] == true){
      total += list2[i];
    }
  }

  switch (total) {
    case 1: //t f f
      blueclose_nopark();
      break;
    case 2: //f t f
      redfar_nopark();
      break;
    case 3: //t t f
      bluefar_nopark();
      break;
    case 4: //f f t
      redclose_park();
      break;
    case 5: //t f t
      blueclose_park();
      break;
    case 6: //f t t
      redfar_park();
      break;
    case 7: //t t t
      bluefar_park();
      break;
    default ://when 0 (f f f)
      redclose_nopark();
  }

}

void autonomous() {
  autonSelector();
  //tempauton();
}
