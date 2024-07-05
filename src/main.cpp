#include "main.h"

// Define drivetrain
pros::Motor frontL{10};
pros::Motor backL{3};
pros::Motor backR{-11};
pros::Motor frontR{-20};

// Define sensors
pros::Imu imu{5};
pros::Rotation vert1{4};
pros::Rotation vert2{-14};
pros::Rotation hor{21};
pros::adi::DigitalOut piston('A');

// Define other stuff
pros::Motor intake1(0);
pros::Motor intake2(0);
pros::Motor intake3(0);
pros::Controller master(pros::E_CONTROLLER_MASTER);

/** @brief Creates a position struct.
 * @param x X value (position.x)
 * @param y Y value (position.y)
 * @param theta Theta value (position.theta)
 */
struct position{
	double x {0};
	double y {0};
	double theta {0};
};

// Global stuff
position pose;
double conversionrate = 4167.77777777777;

/** @brief Position tracking using odometry */
void odom(){

	double prevX = 0, prevY = 0;
	double deltaDistance = 0, prevDistance = 0, distance = 0;
	double deltaHorDistance = 0, prevHorDistance = 0, horDistance = 0;

	vert1.reset_position();
	vert2.reset_position();
	hor.reset_position();

	while (true){
		pose.theta = 360-imu.get_heading();
		distance = (vert1.get_position()+vert2.get_position())/(2*conversionrate);
		horDistance = hor.get_position()/conversionrate;
		
		
		deltaDistance = distance - prevDistance;
		deltaHorDistance = horDistance - prevHorDistance;

		pose.x += deltaDistance*cos(pose.theta);
		pose.y += deltaDistance*sin(pose.theta);

		pose.x -= deltaHorDistance*cos(pose.theta);
		pose.y += deltaHorDistance*sin(pose.theta);

		prevDistance = deltaDistance;
		prevHorDistance = deltaHorDistance;

		pros::lcd::print(0,"X: %.2f",pose.x);
		pros::lcd::print(1,"Y: %.2f",pose.y);
		pros::lcd::print(2,"Theta: %.2f",pose.theta);
		pros::lcd::print(5,":vert1 %.2f",vert1.get_position()/conversionrate);
		pros::lcd::print(6,":vert2 %.2f",vert2.get_position()/conversionrate);
		pros::lcd::print(7,":horizontal %.2f",hor.get_position()/conversionrate);

		pros::delay(10);

	}
}


void stop(){
	frontL.move(0);
	backL.move(0);
	backR.move(0);
	frontR.move(0);
}

/** @brief Turn to an angle
 * @param angle Angle to turn to
 * @param timeout Time to exit movement in milliseconds*/
void turn (double angle, int timeout) {
	// Angular PID constants
	double kP {4};
	double kI {0};
	double kD {0};

	// Angular Variables
	double heading, prevError, errorL, errorR, error, totalError, derivative, MP, startHeading = imu.get_heading();
	// Other stuff

	while (true){
		error = heading - startHeading;
		if (error > 180){
			error -= 360;
		}
		else if (error < -180){
			error += 360;
		}

			totalError += error;
		derivative = prevError - error;
		MP = kP*error + kI*totalError + kD*derivative;

		frontL.move(-MP);
		frontR.move(MP);
		backL.move(MP);
		backR.move(-MP);
	}
	pros::delay(10);
}

/** @brief Longitudal drive movement that uses PID
 * @param distance Distance to move in inches - Negative values will drive backwards
 * @param timeout Time to exit movement in milliseconds
 * @param headingCorrection Toggle heading correction */
void drive(double distance, int timeout,  bool headingCorrection = true){
	vert1.reset_position();
	vert2.reset_position();
	hor.reset_position();
	// Linear PID constants
	double kP {13};
	double kI {0};
	double kD {2};

	// Linear variables
	double error = distance;
	double totalError, prevError, derivative, MP;

	// Angular PID constants
	double akP {5};
	double akI {0};
	double akD {0};

	// Angular variables
	double heading, aerror, atotalError, aprevError, aderivative, aMP, startHeading = imu.get_heading();


	// Lateral PID constants
	double lkP {10};
	double lkI {0};
	double lkD {0};

	// Lateral variables
	double lerror, ltotalError, lprevError, lderivative, lMP;

	// Other Stuff 
	
	// Conditional loop, exits when either the timeout is finished or it gets to the position
	while (fabs(distance - (vert1.get_position()+vert2.get_position())/(2*conversionrate)) > 0.2){
	// while (true) {
		// Different PID calculations
		error = distance - (vert1.get_position()+vert2.get_position())/(2*conversionrate);
		totalError += error;
		derivative = prevError - error;
		MP = kP*error + kI*totalError + kD*derivative;
		
		lerror = (hor.get_position()/conversionrate);
		ltotalError += lerror;
		lderivative = lprevError - lerror;
		lMP = -(lerror*lkP + ltotalError*lkI + lderivative*lkD);

		heading = imu.get_heading();
		aerror = heading-startHeading;
		if (aerror > 180){
			aerror -= 360;
		}
		if (aerror < -180) {
			aerror += 360;
		}
		atotalError += aerror;
		aderivative = aprevError - aerror;
		aMP = aerror*akP + atotalError*akI + aderivative*akD;

		// Movement adds and subtracts the things to make sure the bot stays in the right place
		frontL.move(MP-aMP+lMP);
		backL.move(MP-aMP-lMP);
		backR.move(MP+aMP+lMP);
		frontR.move(MP+aMP-lMP);
		pros::lcd::print(3,"error %.2f", error);
		pros::lcd::print(4," motorpower%.2f", MP);
		// Preverror stuff
		lprevError = lerror;
		aprevError = aerror;
		prevError = error;
		pros::delay(10);
		}
	stop();
}

/** @brief Strafe drive movement that uses PID
 * @param distance Distance to move in inches - Negative values will drive left
 * @param timeout Time to exit movement in milliseconds
 * @param headingCorrection Toggle heading correction
 */
void strafe(double distance, int timeout, bool headingCorrection=true) {
	vert1.reset_position();
	vert2.reset_position();
	hor.reset_position();

	double lkP {12};
	double lkI {0};
	double lkD {0.8};
	double lerror, ltotalError, lprevError, lderivative, lMP;

	double akP {6};
	double akI {0};
	double akD {0};
	double heading, aerror, atotalError, aprevError, aderivative, aMP, startHeading = imu.get_heading();

	double vkP {20};
	double vkI {0};
	double vkD {0};
	double vtotalError, vprevError, vderivative, vMP, verror;

	while (fabs(distance-hor.get_position()/conversionrate) > 0.2 ){
		lerror = distance - hor.get_position()/conversionrate;
		ltotalError += lerror;
		lderivative = lprevError - lerror;
		lMP = lkP*lerror + lkI*ltotalError + lkD*lderivative;
		
		verror = (vert1.get_position()+vert2.get_position())/(2*conversionrate);
		vtotalError += verror;
		vderivative = vprevError - verror;
		vMP = vkP*verror + vkI*vtotalError + vkD*vderivative;

		heading = imu.get_heading();
		aerror = heading-startHeading;
		if (aerror > 180){
			aerror -= 360;
		}
		if (aerror < - 180) {
			aerror += 360;
		}
		atotalError += aerror;
		aderivative = aprevError - aerror;
		aMP = aerror*akP + atotalError*akI + aderivative*akD;

		frontL.move(lMP-aMP-vMP);
		backL.move(-lMP-aMP-vMP);
		backR.move(lMP+aMP-vMP);
		frontR.move(-lMP+aMP-vMP);
		
		vprevError = verror;
		lprevError = lerror;
		aprevError = aerror;
		pros::delay(10);
	}
	stop();
}

void initialize() {
	pros::lcd::initialize();
	frontL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	frontR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::Task tracking(odom);
}

void autonomous() {
	strafe(24,0);
	strafe(-24,0);
	// drive(24,0);
	drive(-24,0);

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
	hor.reset_position();

	bool l2Pressed;
	bool r2Pressed;
	bool l1Pressed;
	bool r1Pressed;
	bool moving;

	while (true) {
		double left = master.get_analog(ANALOG_LEFT_Y);
		double right = master.get_analog(ANALOG_RIGHT_X);
		double rightx = master.get_analog(ANALOG_LEFT_X);

		arcadecontrolx(right, left, rightx);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if (!r1Pressed){
				r1Pressed = true;
				if (moving){
					intake1.move(0);
					intake2.move(0);
					intake3.move(0);
					moving = false;
				}
			}
			else{
				intake1.move(-127);
				intake2.move(-127);
				intake3.move(-127);
				moving = true;
			}
		}
		else{
			r1Pressed = false;
		}
			
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake1.move(127);
			intake2.move(127);
			intake3.move(127);
			moving = true;
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


