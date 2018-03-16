#include "Drive.h"

//NOTE: motor is used when describing methods instead of servo/continuous servo
//		change the comments if it is a necessary thing to do


//To do:
//add imu fusion integration
//add way to calibrate stopVal and speedCalibration through Drive constructor

//constant values
int stopVal = 90; //center value / value that stops the motor
float speedCalibration = (150 - 90) / 100; // (max value - stopping value) / (half of total speed range wanted, i.e. -100 to 100)


void drive(int speed) {//takes an input value from -100(reverse) to 100(forward)
	if (rightServReversed) {
		int lSpeed = stopVal + speedCalibration * speed;	//convert to valid input range of 30(full speed reverse) to 150(full speed forward)
		int rSpeed = stopVal - speedCalibration * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
	}
	else { //same direction of rotation
		int lSpeed = stopVal + speedCalibration * speed;
		int rSpeed = stopVal + speedCalibration * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);

	}
}

void halt() { //Stop the motors
	lServo.write(stopVal);
	rServo.write(stopVal);
}

void centerTurn(int speed) { //turn left if negative, right if positive
	if (rightServReversed) {
		int lSpeed = stopVal + speedCalibration * speed;
		int rSpeed = stopVal + speedCalibration * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
	}
	else {	//same direction of rotation
		int lSpeed = stopVal - speedCalibration * speed; //will turn left if given a negative speed, and right if given a positive speed
		int rSpeed = stopVal + speedCalibration * speed;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
	}
}

void pivotTurn(int speed, int direction) { //turns around the axis of a stopped wheel, direction specifies if pivoting CCW(-1) or CW(+1)
	if (rightServReversed) {
		if (speed < 0) {	//pivoting around right wheel
			int lSpeed = stopVal - speedCalibration * speed * direction;
			lServo.write(lSpeed);		//write speed for left motor
			rServo.write(stopVal);			
		}
		else {				//pivoting around left wheel
			int rSpeed = stopVal + speedCalibration * speed * direction;
			lServo.write(stopVal);			
			rServo.write(rSpeed);		//write speed for right motor
		}
	}
	else {	//direction of rotation is the same
		if (speed < 0) {	//pivoting around right wheel
			int lSpeed = stopVal - speedCalibration * speed * direction;
			lServo.write(lSpeed);		//write speed for left motor
			rServo.write(stopVal);		
		}
		else {				//pivoting around left wheel
			int rSpeed = stopVal - speedCalibration * speed * direction;
			lServo.write(stopVal);		
			rServo.write(rSpeed);		//write speed for right motor
		}
	}
}