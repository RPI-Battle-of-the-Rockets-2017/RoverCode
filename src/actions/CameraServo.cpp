#include "CameraServo.h"

#include "Arduino.h"

#define DELAY_READ      50
#define L_THRESHOLD     100
#define U_THRESHOLD     250
#define ROTATION_VAL    60      //the sign of this value denotes direction
#define STOP_VAL        90
#define INIT_DIRECTION  1

namespace Rover {

void CameraServo::attach(){
    camera.attach(servoPin);
    //pinMode(sensorPin, INPUT);
}

void CameraServo::detach(){
    camera.detach(servoPin);
    //pinMode(sensorPin, INPUT);
}

void CameraServo::resetPosition(){
    if (position == -1) zeroPosition();
    position = 0;
}

void CameraServo::zeroPosition(){
    if (position == -1){
        camera.write(STOP_VAL+INIT_DIRECTION*ROTATION_VAL);

        while(analogRead(sensorPin) >= L_THRESHOLD)
            delayMicroseconds(DELAY_READ);
        while(analogRead(sensorPin) <= U_THRESHOLD)
            delayMicroseconds(DELAY_READ);

        camera.write(STOP_VAL);
        position = 0;
        return;
    }
    moveToPosition(0);
}

void CameraServo::moveToPosition(unsigned short position){
    if (this->position == -1) zeroPosition();
    position %= 4;

    short direction = ((position > this->position) ? 1 : -1);

    while (position != this->position){
        camera.write(STOP_VAL+direction*ROTATION_VAL);

        while(analogRead(sensorPin) >= L_THRESHOLD)
            delayMicroseconds(DELAY_READ);
        while(analogRead(sensorPin) <= U_THRESHOLD)
            delayMicroseconds(DELAY_READ);

        camera.write(STOP_VAL);
        this->position += direction;
    }
}

}
