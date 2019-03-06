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

 void opcontrol() {

   while (true) {
     double power = master.get_analog(ANALOG_LEFT_Y);
     double turn = master.get_analog(ANALOG_RIGHT_X);
     //int left = (int)(pow(((power + turn)/127.0),2.0)*127.0);
     //int right = (int) (pow(((power - turn)/127.0),2.0)*127.0);
     int left = power+turn;
     int right = power-turn;
     left_wheel.move_velocity(left);
     left_chain.move_velocity(left);
     right_wheel.move_velocity(right);
     right_chain.move_velocity(right);
     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) != 0){
       intake.move_velocity(120);
     } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) != 0){
       intake.move_velocity(-120);
     } else{
       intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       intake.move_velocity(0);
     }
     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) != 0){
       angler.move(-120);
     } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) != 0){
       angler.move(120);
     } else{
       angler.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       angler.move_velocity(0);
     }
     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) != 0){
       puncher.move_velocity(120);
     }

     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0){
       left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       left_wheel.move_velocity(0);
       left_chain.move_velocity(0);
       right_wheel.move_velocity(0);
       right_chain.move_velocity(0);
     }else{
       left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
     }

     pros::delay(2);
   }
}
/**
void PIcontrol(float distance, int motor_input){
	const double WHEEL_RADIUS = 5.08;
	const double circumference = WHEEL_RADIUS*2*M_PI;
	bool endreached = false;
	double encoderavg = (IntegratedEncoder(right_wheel) + IntegratedEncoder(left_wheel))/2;
	const double degreeGoal = (distance/circumference)*360.0*GEAR_RATIO;
	double error = degreeGoal - encoderavg;
	double kp = 0.45, kI = 0.0005 , integral_limit = 400.0, motorkP = 0.5;
	double integral = 0.0, encodeDiff = 0.0, riseTimeIteration = 0.0;
	if (distance < 0) { motor_vel *= -1; }
	right_wheel.tare_position();
  left_wheel.tare_position();

	while(!endreached){
		double encoderavg = (IntegratedEncoder(right_wheel) + IntegratedEncoder(left_wheel))/2;
		double endoderdiff = IntegratedEncoder(right_wheel) - IntegratedEncoder(left_wheel);
		if( std::abs(error) < integral_limit) //disable integral unless error is low enough
	{ integral += error; } else { integral = 0; }
		error = degreeGoal - encoderAvg;

    if (std::abs(error*kp) > std::abs(motor_input*3.75)){ //if far enough away keep error a certain value so it's not too big
        error = motor_input/kp;
			}

    motor_vel = std::abs((kp*error)) + std::abs(integral*kI); //PI motor velocity calc

     if (riseTimeIteration < 100){ //to allow for more rise time (starts slowers instead of immediately ramping up)
            riseTimeIteration += 10.0;
            motor_vel = riseTimeIteration;}
			if (std::abs(motor_vel) > motor_input){ motor_vel = motor_input; }//if motor vel too big just set to max motor limit parameter
			if (distance < 0 && motor_vel > 0) { motor_vel *= -1.0; } //makes sure it spins in right direction if reverse

	}
}*/