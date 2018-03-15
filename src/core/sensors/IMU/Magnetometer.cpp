#include "Arduino.h"

#include <Wire.h>
//#include <limits.h>

#include "../IMU.h"
#include "Magnetometer.h"

// Adapted from the Adafruit LSM303 and the Adafruit Unified Sensor librarys

//I2C Address
#define LSM303_ADDRESS_MAG      (0x3C >> 1)     // 0011110x

/**< Gauss to micro-Tesla multiplier */
#define GAUSS_TO_MICROTESLA     (100)

namespace Rover {

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void IMU::Magnetometer::write8(byte reg, byte value){
    IMU::write8(LSM303_ADDRESS_MAG, reg, value);
}
byte IMU::Magnetometer::read8(byte reg){
    return IMU::read8(LSM303_ADDRESS_MAG, reg);
}

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void IMU::Magnetometer::read()
{
    // Read the magnetometer
    Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
    Wire.write(OUT_X_H_M);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);

    // Wait around until enough data is available
    while (Wire.available() < 6);

    // Note high before low (different than accel)
    uint8_t xhi = Wire.read();
    uint8_t xlo = Wire.read();
    uint8_t zhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();

    // Shift values to create properly formed integer (low byte first)
    raw.x = (int16_t)(xlo | ((int16_t)xhi << 8));
    raw.y = (int16_t)(ylo | ((int16_t)yhi << 8));
    raw.z = (int16_t)(zlo | ((int16_t)zhi << 8));
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
IMU::Magnetometer::Magnetometer() {
    _lsm303Mag_Gauss_LSB_XY = 1100.0F;
    _lsm303Mag_Gauss_LSB_Z  = 980.0F;

    autoRangeEnabled = false;

    // Clear the raw mag data
    raw.x = 0;
    raw.y = 0;
    raw.z = 0;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool IMU::Magnetometer::begin()
{
    // Enable I2C
    Wire.begin();

    // Enable the magnetometer
    write8(MR_REG_M, 0x00);

    // LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
    // the default value (0b00010000/0x10)
    uint8_t reg1_a = read8(CRA_REG_M);
    if (reg1_a != 0x10)
    {
        return false;
    }

    // Set the gain to a known level
    setMagGain(GAIN_1_3);

    return true;
}

/**************************************************************************/
/*!
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void IMU::Magnetometer::enableAutoRange(bool enabled)
{
    autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void IMU::Magnetometer::setMagGain(MagGain gain)
{
    write8(CRB_REG_M, (byte)gain);

    this->gain = gain;

    switch(gain)
    {
    case GAIN_1_3:
        _lsm303Mag_Gauss_LSB_XY = 1100;
        _lsm303Mag_Gauss_LSB_Z  = 980;
        break;
    case GAIN_1_9:
        _lsm303Mag_Gauss_LSB_XY = 855;
        _lsm303Mag_Gauss_LSB_Z  = 760;
        break;
    case GAIN_2_5:
        _lsm303Mag_Gauss_LSB_XY = 670;
        _lsm303Mag_Gauss_LSB_Z  = 600;
        break;
    case GAIN_4_0:
        _lsm303Mag_Gauss_LSB_XY = 450;
        _lsm303Mag_Gauss_LSB_Z  = 400;
        break;
    case GAIN_4_7:
        _lsm303Mag_Gauss_LSB_XY = 400;
        _lsm303Mag_Gauss_LSB_Z  = 355;
        break;
    case GAIN_5_6:
        _lsm303Mag_Gauss_LSB_XY = 330;
        _lsm303Mag_Gauss_LSB_Z  = 295;
        break;
    case GAIN_8_1:
        _lsm303Mag_Gauss_LSB_XY = 230;
        _lsm303Mag_Gauss_LSB_Z  = 205;
        break;
    }
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's update rate
*/
/**************************************************************************/
void IMU::Magnetometer::setMagRate(MagRate rate)
{
	byte reg_m = ((byte)rate & 0x07) << 2;
    write8(CRA_REG_M, reg_m);
}


/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool IMU::Magnetometer::getEvent(SensorVec& event) {
  bool readingValid = false;

  while(!readingValid)
  {

    uint8_t reg_mg = read8(SR_REG_Mg);
    if (!(reg_mg & 0x1)) {
			return false;
    }

    /* Read new data */
    read();

    /* Make sure the sensor isn't saturating if auto-ranging is enabled */
    if (!autoRangeEnabled)
    {
      readingValid = true;
    }
    else
    {
#ifdef LSM303_DEBUG
      Serial.print(raw.x); Serial.print(" ");
      Serial.print(raw.y); Serial.print(" ");
      Serial.print(raw.z); Serial.println(" ");
#endif
      /* Check if the sensor is saturating or not */
      if ( (raw.x >= 2040) | (raw.x <= -2040) |
           (raw.y >= 2040) | (raw.y <= -2040) |
           (raw.z >= 2040) | (raw.z <= -2040) )
      {
        /* Saturating .... increase the range if we can */
        switch(gain)
        {
          case GAIN_5_6:
            setMagGain(GAIN_8_1);
            readingValid = false;
#ifdef LSM303_DEBUG
            Serial.println("Changing range to +/- 8.1");
#endif
            break;
          case GAIN_4_7:
            setMagGain(GAIN_5_6);
            readingValid = false;
#ifdef LSM303_DEBUG
            Serial.println("Changing range to +/- 5.6");
#endif
            break;
          case GAIN_4_0:
            setMagGain(GAIN_4_7);
            readingValid = false;
#ifdef LSM303_DEBUG
            Serial.println("Changing range to +/- 4.7");
#endif
            break;
          case GAIN_2_5:
            setMagGain(GAIN_4_0);
            readingValid = false;
#ifdef LSM303_DEBUG
            Serial.println("Changing range to +/- 4.0");
#endif
            break;
          case GAIN_1_9:
            setMagGain(GAIN_2_5);
            readingValid = false;
#ifdef LSM303_DEBUG
            Serial.println("Changing range to +/- 2.5");
#endif
            break;
          case GAIN_1_3:
            setMagGain(GAIN_1_9);
            readingValid = false;
#ifdef LSM303_DEBUG
            Serial.println("Changing range to +/- 1.9");
#endif
            break;
          default:
            readingValid = true;
            break;
        }
      }
      else
      {
        /* All values are withing range */
        readingValid = true;
      }
    }
  }

  event.timestamp = millis();
  event.x = (float)raw.x / _lsm303Mag_Gauss_LSB_XY * GAUSS_TO_MICROTESLA;
  event.y = (float)raw.y / _lsm303Mag_Gauss_LSB_XY * GAUSS_TO_MICROTESLA;
  event.z = (float)raw.z / _lsm303Mag_Gauss_LSB_Z * GAUSS_TO_MICROTESLA;

	return true;
}

}
