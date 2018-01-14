#include "Arduino.h"

#include <Wire.h>
//#include <limits.h>

#include "../IMU.h"
#include "Accelerometer.h"

// Adapted from the Adafruit LSM303 and the Adafruit Unified Sensor librarys

//I2C Address
#define LSM303_ADDRESS_ACCEL    (0x32 >> 1)     // 0011001x

// Constants here to avoid pollution
#define GRAVITY_STANDARD        9.80665F
#define _LSM303ACCEL_MG_LSB     0.001F          // 1, 2, 4 or 12 mg per lsb

namespace Rover {

/***************************************************************************
 PRIVATE FUNCTIONS (Lightly edited)
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void IMU::Accelerometer::write8(byte address, byte reg, byte value)
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
byte IMU::Accelerometer::read8(byte address, byte reg)
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
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void IMU::Accelerometer::read()
{
    // Read the accelerometer
    Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
    Wire.write(OUT_X_L_A | 0x80);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);

    // Wait around until enough data is available
    while (Wire.available() < 6);

    uint8_t xlo = Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t zhi = Wire.read();

    // Shift values to create properly formed integer (low byte first)
    raw.x = (int16_t)(xlo | (xhi << 8)) >> 4;
    raw.y = (int16_t)(ylo | (yhi << 8)) >> 4;
    raw.z = (int16_t)(zlo | (zhi << 8)) >> 4;
}

/***************************************************************************
 CONSTRUCTOR (Lightly edited)
 ***************************************************************************/

IMU::Accelerometer::Accelerometer(){
    // Clear the raw accel data
    raw.x = 0;
    raw.y = 0;
    raw.z = 0;
}

/***************************************************************************
 PUBLIC FUNCTIONS (Somewhat edited)
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool IMU::Accelerometer::begin()
{
  // Enable I2C
  Wire.begin();

  // Enable the accelerometer (100Hz)
  write8(LSM303_ADDRESS_ACCEL, CTRL_REG1_A, 0x57);

  // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
  // if we are connected or not
  uint8_t reg1_a = read8(LSM303_ADDRESS_ACCEL, CTRL_REG1_A);
  if (reg1_a != 0x57)
  {
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool IMU::Accelerometer::getEvent(sensorVec *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensorVec));

  /* Read new data */
  read();

  event->timestamp = millis();
  event->x = (float)raw.x * _LSM303ACCEL_MG_LSB * GRAVITY_STANDARD;
  event->y = (float)raw.y * _LSM303ACCEL_MG_LSB * GRAVITY_STANDARD;
  event->z = (float)raw.z * _LSM303ACCEL_MG_LSB * GRAVITY_STANDARD;

  return true;
}

}
