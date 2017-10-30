#ifndef ROVER_IMU_H
#define ROVER_IMU_H

/**
 * This class is intended to abstractify the IMU (and
 * related measurement units. Whether or not it is needed
 * is another point, but it server to start the base of
 * the backbone to the framework
 */

namespace Rover {

class IMU {
    //Consider exposing these subcomponents to public
    class Accelerometer;
    //insert other classes for other components

    public:
        //Constructor
        IMU();
        //Destructor
        ~IMU();

        Accelerometer* acclerometer
};

}

#endif // ROVER_IMU_H
