#ifndef ROVER_NICHROMECUTTER_H
#define ROVER_NICHROMECUTTER_H

#define HIGH  5
#define LOW  0
#include "Arduino.h"
namespace Rover {
	class NichromeCutter {
	public:
		NichromeCutter(int pinNum) { //figure out how to actually use a pin number
			pin = pinNum;
		}
		void attach() {
			pinMode(pin, INPUT);
		}
		void activate(int time) { //takes an input in milliseconds and activates the nichrome wire for that duration of time
			digitalWrite(pin, HIGH);
			delay(time); //sets pin associated with nichrome to high for a duration
			digitalWrite(pin, LOW);//then low immediately afterwards
		}
	private:
		int pin;
	};
}
#endif // ROVER_NICHROMECUTTER_H
