#include "CameraServo.h"

#include "Arduino.h"

#define DELAY_READ      100
#define L_THRESHOLD     100
#define U_THRESHOLD     250
#define ROTATION_VAL    5      //the sign of this value denotes direction
#define STOP_VAL        92
#define INIT_DIRECTION  -1
#define INIT_TIME		500*1000
#define TIMEOUT         10*1000

namespace Rover {

void CameraServo::attach(){
    camera.attach(servoPin);
    //pinMode(sensorPin, INPUT);
}

void CameraServo::detach(){
    camera.detach();
    //pinMode(sensorPin, INPUT);
}

void CameraServo::initialTurn(){
	attach();
	camera.write(STOP_VAL+INIT_DIRECTION*ROTATION_VAL);
	delayMicroseconds(INIT_TIME);
	camera.write(STOP_VAL);
	detach();
}

void CameraServo::resetPosition(){
    if (position == -1) zeroPosition();
    position = 0;
}

bool CameraServo::zeroPosition(){
    if (position == -1){
        //Start and move the servo
		attach();
        camera.write(STOP_VAL+INIT_DIRECTION*ROTATION_VAL);
        //Record timeout values
        uint32_t start_time = millis();
        bool timed_out;

        //Move until threshold achieved for initial position
        while(analogRead(sensorPin) <= U_THRESHOLD && (timed_out = (millis() - start_time < TIMEOUT)))
            delayMicroseconds(DELAY_READ);

        //Stop the servo and set position
        camera.write(STOP_VAL);
		detach();
        position = 0;
        //handle the case of a time out
        if (timed_out){
            //Do so by moving the servo to the next position, and resetting that to 0
            //or moving back to zero if the timeout is used again.
            if (moveToPosition(1))
                resetPosition();
            else moveToPosition(0);
            //Return false because of timeout
            return false;
        }
        return true;
    }
    //Move to the 0'th position otherwise.
    moveToPosition(0);
}

bool CameraServo::moveToPosition(unsigned short position){
    //Handle positional details
    if (this->position == -1) zeroPosition();
    position %= 4;

    //prepare direction and the bool to hold success value
    short direction = ((position > this->position) ? 1 : -1);	//change this if direction needs to be changed
    bool noTimeOut = true;

    //Start the servo movements
	attach();
    while (position != this->position){
        //Move the servo
        camera.write(STOP_VAL+direction*ROTATION_VAL);

        //Setup time vars
        uint32_t start_time = millis();
        bool timed_out;

        //Move until timeout or until thresholds are achieved
        while(analogRead(sensorPin) >= L_THRESHOLD && (timed_out = (millis() - start_time < TIMEOUT)))
            delayMicroseconds(DELAY_READ);
        while(analogRead(sensorPin) <= U_THRESHOLD && (timed_out = (millis() - start_time < TIMEOUT)))
            delayMicroseconds(DELAY_READ);

        //Stop the camera
        camera.write(STOP_VAL);
        if (timed_out){
            //change the success value, then set the position of the camera to one over the extreme before performing a positional move again.
            //all because of a simple timeout
            noTimeOut = false;
            this->position = (direction > 0) ? 4 : -1;
            direction = ((position > this->position) ? 1 : -1); //also update direction...
            continue;
        }
        //Otherwise increment/decrement the position and continue.
        this->position += direction;
    }
    //stop servo movements.
	detach();

	return noTimeOut;
}

}
