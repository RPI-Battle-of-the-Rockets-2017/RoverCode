#include "ServoUtils.h"

#include "Arduino.h"

//Bread is before read
#define DELAY_BREAD     500*1000
#define DELAY_READ      50
#define TRIGGER         true    //true is high, false is low
#define ROTATION_VAL    60      //the sign of this value denotes direction
#define STOP_VAL        90
#define INIT_DIRECTION  1

namespace Rover {

void CameraServo::attach(){
    camera.attach(servoPin);
    pinMode(sensorPin, INPUT);
}

void CameraServo::resetPosition(){
    if (position == -1) zeroPosition();
    position = 0;
}

void CameraServo::zeroPosition(){
    if (position == -1){
        camera.write(STOP_VAL+INIT_DIRECTION*ROTATION_VAL);

        delayMicroseconds(DELAY_BREAD);
        while(digitalRead(sensorPin) != TRIGGER)
            delayMicroseconds(DELAY_READ);

        camera.write(STOP_VAL);
        tposition = 0;
        return;
    }
    moveToPosition(0);
}

void CameraServo::moveToPosition(short position){
    if (this->position == -1) zeroPosition();
    position %= 4;

    short direction = ((position > this->position) ? 1 : -1);

    while (position != this->position){
        camera.write(STOP_VAL+direction*ROTATION_VAL);

        delayMicroseconds(DELAY_BREAD);
        while(digitalRead(sensorPin) != TRIGGER)
            delayMicroseconds(DELAY_READ);

        camera.write(STOP_VAL);
        this->position += direction;
    }
}

}
