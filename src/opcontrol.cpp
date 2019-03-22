#include "main.h"
#include "cmath"
#include "config.hpp"
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


void setpuncher(){
  puncher.move_relative(-263,1000);
}
void shootpuncher(){
  puncher.move_relative(-80,1000);
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

void turnP(double goal){//simple gyro turn function (positive to right)
  gyro.reset();//sets gyro value to 0
  double error = goal - gyro.get_value() / 10.0; //gyro value is 10 * angle
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
    pros::delay(2);//dont hog cpu
  }//stops movement after turn
  brakeMotors();
  unBrakeMotors();
}



 void moveP(double distance, int multi){//move for distance in cm and multitask accordingly
  double circumference = 5.08 * 2.0 * M_PI;
  double goal = (distance/circumference)*360.0; //make error into degrees

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
  double encoderavg = (right_wheel.get_position() + right_chain.get_position() +  left_wheel.get_position()
                            + left_chain.get_position())/4;
  double error = goal - encoderavg;

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
    encoderavg = (right_wheel.get_position() + right_chain.get_position() +  left_wheel.get_position()
                              + left_chain.get_position())/4;
    error = goal - encoderavg;//recalculate error
    pros::delay(2); //don't hog cpu
  }

  brakeMotors();
  unBrakeMotors();
  angler.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  angler.move_velocity(0);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.move_velocity(0);

}


void opcontrol() {
const int MAXSPEED = 200;
const int TURNAMT = 150;
   while (true) {
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
     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A) != 0){
       arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       arm.move_velocity(-MAXSPEED);
     } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0){
       arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       arm.move_velocity(MAXSPEED);
     }else{
       arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       arm.move_velocity(0);
     }

     /**if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1){
       shootpuncher();
       pros::delay(1000);
       setpuncher();
     } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1){
       setpuncher();
       pros::delay(1000);
     }**/
     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) != 0){
       puncher.move_velocity(-MAXSPEED);
     } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) != 0){
       puncher.move_velocity(MAXSPEED);
     } else{
       puncher.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       puncher.move_velocity(0);
     }

     /**if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 1){
       moveP(60.0, 1); //move one tile while intake out
       turnP(90.0);//turn 90 degrees to the right
     }**/
     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0){
       brakeMotors();
     }else{
       unBrakeMotors();
     }

     pros::delay(2);

   }
}
