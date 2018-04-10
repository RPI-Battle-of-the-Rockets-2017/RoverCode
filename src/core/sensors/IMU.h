#ifndef ROVER_IMU_H
#define ROVER_IMU_H

#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

#include "Arduino.h"
#include "../utilities/Vector.h"

#define sample_size 10

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
	
	void readAccelerometer();
	void readGyroscope();

    //The stored robot states as calculated in this class...
    //Position and orientation
	float filter[sample_size];
	float filter_sum;
	unsigned char shifter;

public:
	
	float accel_x_offset;
	float accel_y_offset;
	float accel_z_offset;
	float gyro_x_offset;
	float gyro_y_offset;
	float gyro_z_offset;
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

    //Gets the current heading of the robot in radians
    //Measured with 0 at magnetic east (think x axis) and increasing counter clockwise.
    //Range is 0 - 2pi
    float getHeading() const { return robotRot.heading; };
	float getPosition() const { return position; };
	
	bool setSleepSettings();
	bool setNormalSettings();
	
	void updateOrientation();
	void resetRates();
	void resetOrientation();

    Accelerometer* accelerometer;
    Magnetometer* magnetometer;
    Gyroscope* gyroscope;
    Barometer* barometer;
	
	unsigned char sensor_counter;
	
	SensorVec vec;
	
	
	
	Vector robotRot;
	Vector robotRotRates;
	Vector robotPrevRotRates;
	Vector rawAccel;
	Vector rawGyro;
	
	float position;
	float velocity;
	float acceleration;
	float prevVelocity;
	float prevAcceleration;
};

}

#include "IMU/Accelerometer.h"
#include "IMU/Magnetometer.h"
#include "IMU/Gyroscope.h"
#include "IMU/Barometer.h"

#endif // ROVER_IMU_H
