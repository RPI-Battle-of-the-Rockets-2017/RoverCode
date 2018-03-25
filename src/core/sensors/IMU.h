#ifndef ROVER_IMU_H
#define ROVER_IMU_H

#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

#include "Arduino.h"
#include "../utilities/Vector.h"

/**
 * This class is intended to abstractify the IMU (and
 * related measurement units. Whether or not it is needed
 * is another point, but it server to start the base of
 * the backbone to the framework
 */

namespace Rover {

class IMU {
private:
    //Consider exposing these subcomponents to public
    class Accelerometer;
    class Magnetometer;
    class Gyroscope;
    class Barometer;
    //insert other classes for other components

    //Helper functions for writing and reading registers over I2C
    static void write8(byte address, byte reg, byte value);
    static byte read8(byte address, byte reg);

    static uint16_t read16(byte address, byte reg);
    static int16_t readS16(byte address, byte reg);

    //The stored robot states as calculated in this class...
    //Position and orientation
    Vector robotPos;
    Vector robotRot;

public:
    typedef struct{
        int32_t timestamp;      /**< time is in milliseconds */
        union {
            Vector v;
            float u[3];
            struct {
                float x;
                float y;
                float z;
            };
            /* Orientation sensors */
            struct {
                float roll;     /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -pi/2<=roll<=pi/2 */
                float pitch;    /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -pi°<=pitch<=pi) */
                float heading;  /**< Angle between the longitudinal axis (the plane body) and magnetic east, measured counterclockwise when viewing from the top of the device. 0-2pi */
            };
        };
    } SensorVec;

    //Constructor
    IMU();
    //Destructor
    ~IMU();

    //Attempt to begin everything that is not yet active.
    //returns true if everything is started, false otherwise.
    bool begin();

    //Get's the current heading of the robot in radians
    //Measured with 0 at magnetic east (think x axis) and increasing counter clockwise.
    //Range is 0 - 2pi
    float getHeading() const { return robotRot.heading; };

    Accelerometer* accelerometer;
    Magnetometer* magnetometer;
    Gyroscope* gyroscope;
    Barometer* barometer;
};

}

#include "IMU/Accelerometer.h"
#include "IMU/Magnetometer.h"
#include "IMU/Gyroscope.h"
#include "IMU/Barometer.h"

#endif // ROVER_IMU_H
