#ifndef ROVER_DRIVE_H
#define ROVER_DRIVE_H

namespace Rover {
	class Drive {
	public:
		Drive(Servo serv1, Servo serv2, bool rIsReversed) {
			lServo = serv1;
			rServo = serv2;
			rightServReversed = rIsReversed;
		}
		void drive(int speed);  //positive for forward, negative for backwards
		void halt();				//
		void centerTurn(int speed); //turns around the center axis of the rover, turn left for negative, right for positive
		void pivotTurn(int speed, int direction); //turns around a wheel, right wheel stationary for negative, left for positive, direction is 1 for rotating CW around a wheel, and -1 for CCW
		//
		const bool rightServReversed; //determines relative orientation of motors to each other
	private:
		Servo lServo;
		Servo rServo;
	};
}

#endif