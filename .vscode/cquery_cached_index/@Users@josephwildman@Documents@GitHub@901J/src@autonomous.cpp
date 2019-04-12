
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
   intake.move_voltage(12000);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 }
 void auton_task(void* param){
   setpuncher();
   shootpuncher();
   pros::delay(100);
   park_PID(35.0, 100, 2);

  /* setpuncher();
   park_PID(30.5, 150, 2);
   park_PID(-3, 50, 2);
   pros::delay(500);
   intakeball();
   intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   intake.move_velocity(0);
   park_PID(-32, 150, 0);
   park_PID(4, 100, 0);
   turn_PID(90, 65);
   doublePunch();
   park_PID(39, 150, 1);
   */
 }

void redclose_nopark(){
  park_PID(30.0, 150, 2);
  pros::delay(1000);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-34, 150, 0);
  park_PID(5, 80, 0);
  turn_PID(-90, 65);
  doublePunch();
  angler.move_velocity(100);
  turn_PID(-3,50);
  park_PID(39, 150, 2);
  angler.move_velocity(0);
  pros::delay(1000);
  park_PID(-25,150,0);
  turn_PID(90,65);
  park_PID(20,100,1);
  turn_PID(-45,60);
  doublePunch();
  park_PID(15,150,0);
}

void redclose_park(){
  park_PID(30.0, 150, 2);
  pros::delay(1000);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-34, 150, 0);
  park_PID(5, 80, 0);
  turn_PID(-90, 65);
  doublePunch();
  angler.move_velocity(100);
  turn_PID(-3,50);
  park_PID(39, 150, 2);
  angler.move_velocity(0);
  pros::delay(1000);
  park_PID(-50,150,0);
  turn_PID(90,65);
  park_PID(20,150,0);
}

void redfar_nopark(){
  park_PID(-35,150,0);
  arm_PID(45,200);
  park_PID(10,150,0);
  turn_PID(120,70);
  park_PID(5,50,5);
  park_PID(-15,150,0);
  turn_PID(90,65);
  park_PID(10,150,2);
  pros::delay(500);
  turn_PID(-45,65);
  doublePunch();
  /*
  park_PID(-35,150,0);
  arm_PID(45,200);
  turn_PID(-10,50);
  park_PID(33,150,5);
  turn_PID(120,80);
  park_PID(31,150,2);
  pros::delay(500);
  turn_PID(-45,65);
  doublePunch();
  */
  /*setpuncher();
  park_PID(30.5, 150, 2);
  park_PID(-3, 50, 2);
  pros::delay(500);
  intakeball();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  turn_PID(-85, 65);
  pros::delay(5000);
  doublePunch();
  brakeMotors();
  unBrakeMotors();*/
}

void redfar_park(){
  park_PID(30.5, 150, 2);
  park_PID(-3, 50, 2);
  turn_PID(-85, 65);
  pros::delay(1000);
  doublePunch();

}

void blueclose_nopark(){
  park_PID(24,150,2);
  pros::delay(100);
  turn_PID(90.0,65);
  doublePunch();
}

void blueclose_park(){
  setpuncher();
  park_PID(30.5, 150, 2);
  park_PID(-3, 50, 2);
  pros::delay(500);
  intakeball();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-32, 150, 0);
  park_PID(4, 100, 0);
  turn_PID(90, 65);
  doublePunch();
  park_PID(-20,100,0);
  turn_PID(-90,65);
  park_PID(36,200,1);
  brakeMotors();
  unBrakeMotors();
}

void bluefar_nopark(){
  setpuncher();
  park_PID(30.5, 150, 2);
  park_PID(-3, 50, 2);
  pros::delay(500);
  intakeball();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  turn_PID(85, 65);
  pros::delay(5000);
  doublePunch();
  brakeMotors();
  unBrakeMotors();
}

void bluefar_park(){
  setpuncher();
  park_PID(30.5, 150, 2);
  park_PID(-3, 50, 2);
  pros::delay(500);
  intakeball();
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  turn_PID(85, 65);
  pros::delay(5000);
  doublePunch();
  turn_PID(-85, 65);
  park_PID(-5,65,0);
  turn_PID(90,65);
  park_PID(10,200,1);
  brakeMotors();
  pros::delay(500);
  unBrakeMotors();
}

void autonomous(){
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
