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

}
