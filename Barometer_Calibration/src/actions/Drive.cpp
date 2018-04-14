#include "Drive.h"
//NOTE: motor is used when describing methods instead of servo/continuous servo
//		change the comments if it is a necessary thing to do

//constant values
#define STOP_VAL 93 //center value / value that stops the motor
#define SPEED_CALIBRATION ((150 - STOP_VAL) / 100.0) // (max value - stopping value) / (half of total speed range wanted, i.e. -100 to 100)
#define BEARING_CALIBRATION  0.06 //Sets the tolerance range (approximately 1% of 2 PI)
#define PI_VAL  3.14159265 //PI

namespace Rover {
	void Drive::setHeading(float nHeading) {//set a new desired Heading
		desiredHeading = nHeading;
	}

	void Drive::drive(int speed) {//takes an input value from -100(reverse) to 100(forward)
		//attach();
		correction = desiredHeading - imu->getHeading(); //obtain a new correction for each call to the drive function
		if (correction > PI_VAL) { //correct for moving at max one pi radians in either direction 
			correction = correction - 2 * PI_VAL;
		}
		else if (correction < -1 * PI_VAL) {
			correction = correction + 2 * PI_VAL;
		}
		int lSpeed;
		int rSpeed;
		if (rightServReversed) {
			lSpeed = SPEED_CALIBRATION * speed;	//convert to valid input range of 30(full speed reverse) to 150(full speed forward)
			rSpeed = -1 * SPEED_CALIBRATION * speed;
		}
		else { //same direction of rotation
			lSpeed = SPEED_CALIBRATION * speed;	//convert to valid input range of 30(full speed reverse) to 150(full speed forward)
			rSpeed = SPEED_CALIBRATION * speed;
		}
		lSpeed *= (correction < BEARING_CALIBRATION ? 1 : ((PI_VAL + correction) / PI_VAL));		//correct for how far off direction we are supposed to head in
		rSpeed *= (correction > -1 * BEARING_CALIBRATION ? 1 : ((PI_VAL - correction) / PI_VAL));	//by modifying the speed of either motor
		lSpeed += STOP_VAL;			//add midpoint to get appropriate range for motors
		rSpeed += STOP_VAL;
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
	}

	void Drive::halt() { //Stop the motors
		lServo.write(STOP_VAL);
		rServo.write(STOP_VAL);
		//detach();
	}
	
	void Drive::detach() {
		lServo.detach();
		rServo.detach();
	}

	void Drive::centerTurn(int speed) { //turn left if negative, right if positive
		//attach();
		int lSpeed;
		int rSpeed;
		if (rightServReversed) {
			lSpeed = STOP_VAL - SPEED_CALIBRATION * speed;
			rSpeed = STOP_VAL - SPEED_CALIBRATION * speed;
		}
		else {	//same direction of rotation
			lSpeed = STOP_VAL + SPEED_CALIBRATION * speed; //will turn left if given a negative speed, and right if given a positive speed
			rSpeed = STOP_VAL - SPEED_CALIBRATION * speed;
		}
		lServo.write(lSpeed);			//write speed for both motors
		rServo.write(rSpeed);
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
