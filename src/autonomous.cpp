
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
 void close_double_task(void * param){
   intake.move_velocity(-150);
   pros::delay(250);
   angler.move_velocity(0);
   intake.move_velocity(0);
   doublePunch();
   angler.move_velocity(0);
   intake.move_velocity(0);
 }
 void intakeball(){
   intake.move_voltage(12000);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 }
 void auton_task(void* param){
   setpuncher();
   shootpuncher();
   pros::delay(140);
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
  park_PID(34.5, 180, 2);
  pros::delay(200);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-53.0, 160, 0);
  park_PID(8, 150, 0);
  turn_PID(-90, 65);
  std::string text("doublepunch");
  pros::Task punchMove(close_double_task,&text);
  pros::delay(1000);
  park_PID(48, 150, 2);
  park_PID(-30,150,2);
  pros::delay(100);
  turn_PID(90,60);
  park_PID(-5,160,0);
  pros::delay(100);
  park_PID(33,100,1);
  pros::delay(100);
  turn_PID(-45,60);
  shootpuncher();
  park_PID(34,200,0);
  /*park_PID(34, 180, 2);
  pros::delay(150);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-53.0, 160, 0);
  park_PID(8, 150, 0);
  turn_PID(-88, 75);
  intake.move_velocity(-150);
  pros::delay(250);
  angler.move_velocity(0);
  intake.move_velocity(0);
  doublePunch();
  turn_PID(-1.5,40);
  park_PID(45, 200, 2);
  park_PID(-30,150,2);
  pros::delay(100);
  turn_PID(90,60);
  park_PID(-5,120,2);
  pros::delay(300);
  park_PID(38,150,1);
  pros::delay(100);
  turn_PID(-55,60);
  shootpuncher();
  park_PID(30,200,0);
  */
}

void redclose_park(){
  park_PID(34.5, 200, 2);
  pros::delay(150);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-53.0, 160, 0);
  park_PID(9, 150, 0);
  turn_PID(-91, 70);
  intake.move_velocity(-150);
  pros::delay(250);
  angler.move_velocity(0);
  intake.move_velocity(0);
  doublePunch();
  park_PID(48, 200, 2);
  angler.move_velocity(0);
  park_PID(-74,150,0);
  turn_PID(90,60);
  park_PID(30,200,0);
}

void redfar_nopark(){
  park_PID(-52,180,0);
  arm_PID(90,200);
  park_PID(22.5,150,0);
  pros::delay(100);
  turn_PID(-110,50);
  pros::delay(200);
  park_PID(12.5,80,5);
  park_PID(-26,150,0);
  pros::delay(100);
  turn_PID(-86,60);
  pros::delay(100);
  park_PID(24,100,2);
  pros::delay(500);
  park_PID(-5,60,0);
  pros::delay(100);
  turn_PID(-41,65);
  pros::delay(200);
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

void arm_stack_task2(void* param){
  arm_PID(135,200);
  pros::delay(300);
  arm_PID(-10,200);

}
void redfar_park(){

  park_PID(-52,200,0);
  arm_PID(90,200);
  park_PID(22,170,0);
  pros::delay(50);
  turn_PID(-108,75);
  pros::delay(100);
  park_PID(11.6,95,5);
  park_PID(-26,180,0);
  pros::delay(50);
  turn_PID(-84,65);
  pros::delay(100);
  park_PID(22.2,180,2);
  park_PID(-10,180,2);
  pros::delay(50);
  turn_PID(-80,60);
  pros::delay(50);
  park_PID(36,190,0);
  pros::delay(200);


  /*park_PID(-35,150,0);
  arm_PID(45,200);
  park_PID(7,150,0);
  turn_PID(120,70);
  park_PID(10,50,5);
  park_PID(-15,150,0);
  turn_PID(90,65);
  park_PID(12,150,2);
  pros::delay(500);
  turn_PID(-45,65);
  doublePunch();
  turn_PID(-45,65);
  park_PID(12,150,2);*/
}

void blueclose_nopark(){
  park_PID(34.5, 200, 2);
  pros::delay(150);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-53.0, 160, 0);
  park_PID(8, 150, 0);
  turn_PID(88, 60);
  intake.move_velocity(-150);
  pros::delay(250);
  angler.move_velocity(0);
  intake.move_velocity(0);
  doublePunch();
  turn_PID(1.5,40);
  park_PID(45, 200, 2);
  angler.move_velocity(0);
  park_PID(-30,150,2);
  pros::delay(100);
  turn_PID(-90,60);
  park_PID(-7.5,120,2);
  pros::delay(300);
  park_PID(38,200,1);
  pros::delay(100);
  turn_PID(59,60);
  shootSensor();
  park_PID(30,200,0);
}

void blueclose_park(){
  park_PID(34.5, 200, 2);
  pros::delay(150);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);
  park_PID(-53.0, 160, 0);
  park_PID(8, 150, 0);
  turn_PID(88, 60);
  intake.move_velocity(-150);
  pros::delay(250);
  angler.move_velocity(0);
  intake.move_velocity(0);
  doublePunch();
  turn_PID(1.5,40);
  park_PID(45, 200, 2);
  angler.move_velocity(0);
  turn_PID(-1.5,40);
  park_PID(-54,150,0);
  turn_PID(-90,60);
  park_PID(20,150,0);
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

  shootSensor();
  pros::delay(10000);
}

void autonomous(){
  if(blueSide == false)//if red
   {
     if(farSide == false){
       if(park == true){
          redclose_park();
        }
        else{
          redclose_nopark();
        }
     }
       else{ //far side
         if(park == true){
            redfar_park();
          }
          else{
            redfar_nopark();
          }
     }
   }
 else {//blue side chosen
   if(farSide == false){
     if(park == true){
        blueclose_park();
      }
      else{
        blueclose_nopark();
      }
   }
     else{ //far side
       if(park == true){
          bluefar_park();
        }
        else{
          bluefar_nopark();
        }
   }


}
}
