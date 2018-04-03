#ifndef ROVER_CAMERASERVO_H
#define ROVER_CAMERASERVO_H

#include <Servo.h>

/**
 * Consider using this file to isolate the servo control outside of
 * the main code as well as adding a few useful functions that can
 * make the servos easier to use
 */

namespace Rover {

//or call Servo, and wrap the servo library into here. Allows easier changing.
class CameraServo {
public:
    CameraServo(int servoPin, int sensorPin, bool reversed = false)
        : servoPin(servoPin), sensorPin(sensorPin), reversed(reversed), position(-1) {};

    void attach();

    short getPosition() const { return position; };

    void resetPosition();

    void zeroPosition();

    void moveToPosition(short position);

    void nextPosition() { moveToPosition(position++); };

    const bool reversed;
private:
    const int servoPin, sensorPin;
    Servo camera;
    /*
    Positions:
    -1: undefined (first call to zero, reset, or move will move the servo to initial position 0)
    0:  Direction 1
    1:  Direction 2
    2:  Direction 3
    3:  Direction 4
    input positions are modulo'd to keep everything in range.
    */
    short position;
};

}
#endif // ROVER_SERVOUTILS_H
