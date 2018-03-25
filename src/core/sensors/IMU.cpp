#include "IMU.h"

namespace Rover {

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void IMU::write8(byte address, byte reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte IMU::read8(byte address, byte reg)
{
    byte value;

    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t IMU::read16(byte address, byte reg)
{
    uint16_t value;

    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)2);
    value = (Wire.read() << 8) | Wire.read();
    Wire.endTransmission();

    return value;
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
int16_t IMU::readS16(byte address, byte reg)
{
  uint16_t i = read16(address, reg);
  return (int16_t)i;
}

IMU::IMU(){
    robotPos.x = 0;
    robotPos.y = 0;
    robotPos.z = 0;

    robotRot.roll = 0;
    robotRot.pitch = 0;
    robotRot.heading = 0;

    accelerometer = new Accelerometer();
    magnetometer = new Magnetometer();
    gyroscope = new Gyroscope();
    barometer = new Barometer();
}

IMU::~IMU(){
    delete accelerometer;
    delete magnetometer;
    delete gyroscope;
    delete barometer;
}

//Attempt to begin everything that is not yet active.
//returns true if everything is started, false otherwise.
bool IMU::begin(){
    bool success = true;
    if (!accelerometer->active()) success = (accelerometer->begin()) ? success : false;
    if (!magnetometer->active()) success = (magnetometer->begin()) ? success : false;
    if (!gyroscope->active()) success = (gyroscope->begin()) ? success : false;
    if (!barometer->active()) success = (barometer->begin()) ? success : false;
    return success;
}

}
