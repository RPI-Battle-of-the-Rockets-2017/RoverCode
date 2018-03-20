#include "Drive.h"

//NOTE: motor is used when describing methods instead of servo/continuous servo
//		change the comments if it is a necessary thing to do


//To do:
//add imu fusion integration
//add way to calibrate stopVal and speedCalibration through Drive constructor

//constant values
#define STOP_VAL 90 //center value / value that stops the motor
#define SPEED_CALIBRATION ((150 - 90) / 100.0) // (max value - stopping value) / (half of total speed range wanted, i.e. -100 to 100)

namespace Rover {

void Drive::drive(int speed) {//takes an input value from -100(reverse) to 100(forward)
	if (rightServReversed) {
		int lSpeed = STOP_VAL + SPEED_CALIBRATION * speed;	//convert to valid input range of 30(full speed reverse) to 150(full speed forward)
		int rSpeed = STOP_VAL - SPEED_CALIBRATION * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
	}
	else { //same direction of rotation
		int lSpeed = STOP_VAL + SPEED_CALIBRATION * speed;
		int rSpeed = STOP_VAL + SPEED_CALIBRATION * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);

	}
}

void Drive::halt() { //Stop the motors
	lServo.write(STOP_VAL);
	rServo.write(STOP_VAL);
}

void Drive::centerTurn(int speed) { //turn left if negative, right if positive
	if (rightServReversed) {
		int lSpeed = STOP_VAL - SPEED_CALIBRATION * speed;
		int rSpeed = STOP_VAL - SPEED_CALIBRATION * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
	}
	else {	//same direction of rotation
		int lSpeed = STOP_VAL + SPEED_CALIBRATION * speed; //will turn left if given a negative speed, and right if given a positive speed
		int rSpeed = STOP_VAL - SPEED_CALIBRATION * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
	}
}

void Drive::pivotTurn(int speed, bool direction) { //turns around the axis of a stopped wheel, direction specifies if pivoting CCW(-1) or CW(+1)
	if (rightServReversed) {
		if (speed < 0) {	//pivoting around right wheel
			int lSpeed = STOP_VAL - SPEED_CALIBRATION * speed * (direction ? 1 : -1);
			lServo.write(lSpeed);		//write speed for left motor
			rServo.write(STOP_VAL);
		}
		else {				//pivoting around left wheel
			int rSpeed = STOP_VAL - SPEED_CALIBRATION * speed * (direction ? 1 : -1);
			lServo.write(STOP_VAL);
			rServo.write(rSpeed);		//write speed for right motor
		}
	}
	else {	//direction of rotation is the same
		if (speed < 0) {	//pivoting around right wheel
			int lSpeed = STOP_VAL - SPEED_CALIBRATION * speed * (direction ? 1 : -1);
			lServo.write(lSpeed);		//write speed for left motor
			rServo.write(STOP_VAL);
		}
		else {				//pivoting around left wheel
			int rSpeed = STOP_VAL + SPEED_CALIBRATION * speed * (direction ? 1 : -1);
			lServo.write(STOP_VAL);
			rServo.write(rSpeed);		//write speed for right motor
		}
	}
}

}
