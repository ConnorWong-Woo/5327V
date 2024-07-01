#include "main.h"

// Define motors
pros::Motor frontL{-17};
pros::Motor frontR{12};
pros::Motor backL{-21};
pros::Motor backR{4};

// Define sensors
pros::Imu imu{8};
pros::Rotation vert1{-16};
pros::Rotation vert2{19};
pros::Rotation hor{6};


pros::Controller master(pros::E_CONTROLLER_MASTER);

double conversionrate = 4167.7777;

void ide(){
	vert1.reset_position();
	vert2.reset_position();
	while (true){
		pros::lcd::print(1,"figgdddddo2t: %.2f", (vert1.get_position())/conversionrate);
		pros::lcd::print(2,"figgdddddo2t: %.2f", (vert2.get_position())/conversionrate);
		pros::delay(25);
		}
}

/** @brief Linear drive movement that uses PID
 * @param distance Distance to move in inches
 * @param timeout Time to exit movement in milliseconds
 * @param maxError Maximum allowed error in inches
 * @param headingCorrection Toggle heading correction
 * @param debug Print out helpful information for debugging
 */
void lindrive(double distance, int timeout, double maxError, bool headingCorrection = true, bool debug = false){
	vert1.reset_position();
	vert2.reset_position();
	hor.reset_position();
	// Linear PID constants
	double kP {8};
	double kI {0};
	double kD {0.4};

	// Linear variables
	double error = distance;
	double totalError, prevError, derivative;

	// Angular PID constants
	double akP {1};
	double akI {0};
	double akD {0};

	// Angular variables
	double startHeading, heading, aerror, atotalError, aprevError, aderivative;


	// Lateral PID constants
	double lkP {3};
	double lkI {0};
	double lkD {0};

	// Angular variables
	double lerror, ltotalError, lprevError, lderivative;
	double leftMotor, rightMotor;

	// Other Stuff 
	// int startTime {pros::millis()}; // Start timer
	double motorPower;
	
	// Main loop  
	while (fabs(distance - (vert1.get_position()+vert2.get_position())/(2*conversionrate)) > maxError){ // && pros::millis()-startTime < timeout
		error = fabs(distance - (vert1.get_position()+vert2.get_position())/(2*conversionrate));
		totalError += error;
		derivative = prevError - error;
		
		lerror = hor.get_position()/conversionrate;

		// heading = imu.get_heading();
		// aerror = startHeading-heading;
		// if (aerror < 0){
		// 	aerror += 360;
		// }
		// atotalError += aerror;
		// aderivative = aprevError - aerror;

		motorPower = kP*error + kI*totalError + kD*derivative;
		frontL.move(motorPower);
		frontR.move(motorPower);
		backL.move(motorPower);
		backR.move(motorPower);

		prevError = error;
		// aprevError = error;

		if (debug){
			pros::lcd::print(3,"Error: %.2f", error);
			// pros::lcd::print(4,"prevError: %.2f", prevError);
			// pros::lcd::print(5,"totalError: %.2f", totalError);
			// pros::lcd::print(6,"derivative: %.2f", derivative);
			pros::lcd::print(7,"motorPower: %.2f", motorPower);
		}
		pros::delay(25);
	}
}

void initialize() {
	pros::lcd::initialize();
	frontL.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	backL.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	frontR.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	backR.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	pros::Task nifag(ide);
}

void autonomous() {
	lindrive(24,30000,0,false,true);

}



void arcadecontrolx(double angular, double lateral, double lateralx)
{
	
	frontL.move(lateral+angular+lateralx);
	backL.move(lateral+angular-lateralx);
	backR.move(lateral-angular+lateralx);
	frontR.move(lateral-angular-lateralx);

}


void opcontrol() {
	vert1.reset_position();
	vert2.reset_position();
	bool l2Pressed;
	bool r2Pressed;
	bool l1Pressed;
	bool r1Pressed;
	pros::Motor intake1(14);
	pros::Motor intake2(-15);
	pros::Motor intake3(20);
	pros::adi::DigitalOut piston('A');

	while (true) {
		double left = master.get_analog(ANALOG_LEFT_Y);
		double right = master.get_analog(ANALOG_RIGHT_X);
		double rightx = master.get_analog(ANALOG_LEFT_X);

		arcadecontrolx(right, left, 0);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake1.move(0);
			intake2.move(0);
			intake3.move(0);
			}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake1.move(127);
			intake2.move(127);
			intake3.move(127);
		}
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			piston.set_value(1);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			piston.set_value(0);
		}

		pros::delay(25);

	}
}


