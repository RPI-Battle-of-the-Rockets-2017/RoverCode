#ifndef ROVER_DRIVE_H
#define ROVER_DRIVE_H

#include <Servo.h>

namespace Rover {
	class Drive {
	public:
		Drive(int serv1, int serv2, bool rIsReversed = true) : rightServReversed(rIsReversed) {
			s1 = serv1; s2 = serv2;
		}

		void attach(){
		    lServo.attach(s1);
			rServo.attach(s2);
		}
		//Speed is from -100 to 100
		//positive for forward, negative for backwards
		void drive(int speed);
		void halt();
		//turns around the center axis of the rover, turn left for positive, right for negative
		void centerTurn(int speed);
		//turns around the inside wheel, left is positive, right is negative
		//direction is true for forwards, false for backwards
		void pivotTurn(int speed, bool direction);
		//
		const bool rightServReversed; //determines relative orientation of motors to each other
	private:
	    int s1, s2;
		Servo lServo;
		Servo rServo;
	};
}

#endif