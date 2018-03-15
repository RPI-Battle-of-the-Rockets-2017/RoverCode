#ifndef ROVER_IMU_H
#define ROVER_IMU_H

#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

#include "Arduino.h"
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

public:
    typedef struct{
        int32_t timestamp;      /**< time is in milliseconds */
        union {
            float v[3];
            struct {
                float x;
                float y;
                float z;
            };
            /* Orientation sensors */
            struct {
                float roll;     /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90°<=roll<=90° */
                float pitch;    /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180°<=pitch<=180°) */
                float heading;  /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359° */
            };
        };
    } SensorVec;

    //Constructor
    IMU();
    //Destructor
    ~IMU();

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
