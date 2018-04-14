#include "Arduino.h"

#include <Wire.h>
//#include <limits.h>

#include "../IMU.h"
#include "Accelerometer.h"

// Adapted from the Adafruit LSM303 and the Adafruit Unified Sensor libraries

//I2C Address
#define LSM303_ADDRESS_ACCEL    (0x32 >> 1)     // 0011001x

// Constants here to avoid pollution
#define GRAVITY_STANDARD        9.80665F
//#define _LSM303ACCEL_MG_LSB     0.001F          // 1, 2, 4 or 12 mg per lsb

namespace Rover {

/***************************************************************************
 PRIVATE FUNCTIONS (Lightly edited)
 ***************************************************************************/
inline void IMU::Accelerometer::write8(byte reg, byte value){
    IMU::write8(LSM303_ADDRESS_ACCEL, reg, value);
}

inline byte IMU::Accelerometer::read8(byte reg){
    return IMU::read8(LSM303_ADDRESS_ACCEL, reg);
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
    // TODO: actually look up the LSB values
    accel_LSB = 0.002F;

    begun = false;

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
  write8(CTRL_REG1_A, 0x57);

  // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
  // if we are connected or not
  uint8_t reg1_a = read8(CTRL_REG1_A);
  if (reg1_a != 0x57)
  {
    return false;
  }

  return begun = true;
}

/**************************************************************************/
/*!
    @brief  Sets up the accelerometer for low power mode and arduino sleep
*/
/**************************************************************************/
bool IMU::Accelerometer::setSleepSettings()
{
    // Sets data rate to 400Hz, enables low power mode
    write8(CTRL_REG1_A, 0x7F);
    // Verify register set properly
    if (read8(CTRL_REG1_A) != 0x7F) return false;

    //Enables AOI1 interrupt on INT1
    write8(CTRL_REG3_A, 0x40);
    // Verify register set properly
    if (read8(CTRL_REG3_A) != 0x40) return false;

    //Sets scale to +/- 4g
    write8(CTRL_REG4_A, 0x10);
    // Verify register set properly
    if (read8(CTRL_REG4_A) != 0x10) return false;

    //Sets interrupt to active low
    write8(CTRL_REG6_A, 0x02);
    // Verify register set properly
    if (read8(CTRL_REG6_A) != 0x02) return false;

    //Enables interrupt generation for high X OR high Y OR high Z
    write8(INT1_CFG_A, 0x2A);
    // Verify register set properly
    if (read8(INT1_CFG_A) != 0x2A) return false;

    //Sets interrupt threshold to 2.5g
    write8(INT1_THS_A, 0x30);	//FOR THE LOVE OF GOD, CHANGE THIS BACK TO 0x50
    // Verify register set properly
    if (read8(INT1_THS_A) != 0x30) return false;

    //Sets minimum duration for interrupt trigger to 100ms (5/50)
    write8(INT1_DURATION_A, 0x19);
    // Verify register set properly
    if (read8(INT1_DURATION_A) != 0x19) return false;

    return true;
}

/**************************************************************************/
/*!
    @brief  Sets up the accelerometer for normal operation
*/
/**************************************************************************/
bool IMU::Accelerometer::setNormalSettings()
{
    //Sets data rate to 400Hz, disables low power mode
    write8(CTRL_REG1_A, 0x77);
    // Verify register set properly
    if (read8(CTRL_REG1_A) != 0x77) return false;

    return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool IMU::Accelerometer::getEvent(SensorVec & event) {
  /* Read new data */
  read();

  event.timestamp = millis();
  event.x = (float)raw.x * accel_LSB * GRAVITY_STANDARD;
  event.y = (float)raw.y * accel_LSB * GRAVITY_STANDARD;
  event.z = (float)raw.z * accel_LSB * GRAVITY_STANDARD;

  return true;
}

}
