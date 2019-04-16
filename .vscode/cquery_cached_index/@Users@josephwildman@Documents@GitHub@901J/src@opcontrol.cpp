#include "main.h"
#include "cmath"
#include "config.hpp"
#include "auton_functions.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 /**
 #define LEFT_WHEEL_PORT 17
 #define LEFT_CHAIN_PORT 18
 #define RIGHT_CHAIN_PORT 14
 #define RIGHT_WHEEL_PORT 15
 #define INTAKE_MOTOR_PORT 9
 #define PUNCHER_MOTOR_PORT 20
 #define ANGLER_MOTOR_PORT 11
 pros::Motor left_wheel (LEFT_WHEEL_PORT);
 pros::Motor right_wheel (RIGHT_WHEEL_PORT, true);
 pros::Motor left_chain (LEFT_CHAIN_PORT);
 pros::Motor right_chain (RIGHT_CHAIN_PORT, true);
 pros::Controller master (CONTROLLER_MASTER);
**/

void puncher_task(void* param){
  const int MAXSPEED = 200;

  while(true){

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) != 0){
      //slewRateControl(&puncher,-200,30);
      shootpuncher();

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) != 0){
      //slewRateControl(&puncher,200,30);
      doublePunch();

    } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) != 0){
      setpuncher();

    }else if((master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0) || master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
      shootSensor();
      }else {
      puncher.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      puncher.move_velocity(0);
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) != 0){
      intake.move_velocity(-MAXSPEED);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) != 0){
      intake.move_velocity(MAXSPEED);
    } else{
      intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake.move_velocity(0);
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) != 0){
      angler.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      angler.move_velocity(-MAXSPEED);
    } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) != 0){
      angler.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      angler.move_velocity(MAXSPEED);
    } else{
      //angler.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      angler.move_velocity(0);
    }
    pros::delay(5);
  }

}




void opcontrol() {
const int MAXSPEED = 200;
const int TURNAMT = 150;
uint32_t encoder_value = 0;
std::string text("puncher");
pros::Task punchTask(puncher_task,&text);

/*
move_PID(50.0,180.0,0);
turn_PID(180.0,100);
move_PID(50.0,180.0,0);
turn_PID(-90.0,100);
turn_PID(-90.0,90);
move_PID(50.0,200.0,0);
move_PID(-40.0,180.0,0);
pros::delay(300);
turn_PID(90.0,70);
*/

   while (true) {

     /*double currentPosition = (gyro.get_value()+gyro2.get_value())/2;
     printf("gyro avg %f\r\n",currentPosition);
     currentPosition = gyro.get_value();
     printf("gyro 1 %f\r\n",currentPosition);
     currentPosition = gyro2.get_value();
     printf("gyro 2 %f\r\n",currentPosition);
     */
     double power = MAXSPEED * master.get_analog(ANALOG_LEFT_Y) / 127;
     double turn = MAXSPEED * master.get_analog(ANALOG_RIGHT_X) / TURNAMT;//127 max, increase for less turn

     int left = (int)(pow(((power + turn)/(MAXSPEED*1.0)),2.0)*(MAXSPEED*1.0));
     int right = (int) (pow(((power - turn)/(MAXSPEED*1.0)),2.0)*(MAXSPEED*1.0));


     if( (power+turn) < 0){//makes sure left and right values are pos/neg
       left *= -1;
     }
     if( (power - turn) < 0){
       right *= -1;
     }

     //arcade drive

     left_wheel.move_velocity(left);
     left_chain.move_velocity(left);
     right_wheel.move_velocity(right);
     right_chain.move_velocity(right);

     /**
     slewRateControl(&left_wheel, left);
     slewRateControl(&left_chain, left);
     slewRateControl(&right_wheel, right);
     slewRateControl(&right_chain, right);
     **/




     /**if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1){
       shootpuncher();
       pros::delay(1000);
       setpuncher();
     } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1){
       setpuncher();
       pros::delay(1000);
     }**/
     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A) != 0){
       slewRateControl(&arm,200,DEFAULTSLEWRATEINCREMENT);
     } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0){
       slewRateControl(&arm,-200,DEFAULTSLEWRATEINCREMENT);
     }
     else {
       arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       arm.move_velocity(0);
     }


     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0){
       brakeMotors();
     }else{
       unBrakeMotors();
     }



     pros::delay(10);

   }
}
