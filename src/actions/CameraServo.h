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

    //Get the position of camera servo
    short getPosition() const { return position; };

    //make the current position of the servo 0, unless it's -1, where it'll just zero the position.
    void resetPosition();

    //Rotate the camera servo to it's zero position
    void zeroPosition();

    //Move the camera servo to the position passed in. It is modulo'd by 4 to keep it in range.
    void moveToPosition(unsigned short position);

    //Move the camera to the next incremental position, wrapping 4 around to 0
    void nextPosition() { moveToPosition(position++); };

    //Publicly accesible reversed bool
    const bool reversed;
private:
    //Attach or detach the servo to the passed in pin.
    void attach();
    void detach();
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
