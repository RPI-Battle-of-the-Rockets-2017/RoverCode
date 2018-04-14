#include "Arduino.h"

#include <Wire.h>

#include "../IMU.h"
#include "Barometer.h"

#define BMP085_ADDRESS                (0x77)
//#define BMP085_USE_DATASHEET_VALS /* Uncomment for sanity check */


namespace Rover{

/***************************************************************************
 PRIVATE FUNCTIONS (Lightly edited)
 ***************************************************************************/
inline void IMU::Barometer::writeCommand(byte reg, byte value){
    IMU::write8(BMP085_ADDRESS, reg, value);
}

inline byte IMU::Barometer::read8(byte reg){
    return IMU::read8(BMP085_ADDRESS, reg);
}

inline uint16_t IMU::Barometer::read16(byte reg){
    return IMU::read16(BMP085_ADDRESS, reg);
}

inline int16_t IMU::Barometer::readS16(byte reg){
    return IMU::readS16(BMP085_ADDRESS, reg);
}
/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void IMU::Barometer::readCoefficients()
{
  #if BMP085_USE_DATASHEET_VALS
    BMPcoeffs.ac1 = 408;
    BMPcoeffs.ac2 = -72;
    BMPcoeffs.ac3 = -14383;
    BMPcoeffs.ac4 = 32741;
    BMPcoeffs.ac5 = 32757;
    BMPcoeffs.ac6 = 23153;
    BMPcoeffs.b1  = 6190;
    BMPcoeffs.b2  = 4;
    BMPcoeffs.mb  = -32768;
    BMPcoeffs.mc  = -8711;
    BMPcoeffs.md  = 2868;
    BMPMode        = 0;
  #else
    BMPcoeffs.ac1 = readS16(CAL_AC1);
    BMPcoeffs.ac2 = readS16(CAL_AC2);
    BMPcoeffs.ac3 = readS16(CAL_AC3);
    BMPcoeffs.ac4 = read16(CAL_AC4);
    BMPcoeffs.ac5 = read16(CAL_AC5);
    BMPcoeffs.ac6 = read16(CAL_AC6);
    BMPcoeffs.b1 = readS16(CAL_B1);
    BMPcoeffs.b2 = readS16(CAL_B2);
    BMPcoeffs.mb = readS16(CAL_MB);
    BMPcoeffs.mc = readS16(CAL_MC);
    BMPcoeffs.md = readS16(CAL_MD);
  #endif

}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void IMU::Barometer::readRawTemperature(int32_t *temperature)
{
  #if BMP085_USE_DATASHEET_VALS
    *temperature = 27898;
  #else
    uint16_t t;
    writeCommand(CONTROL, READTEMPCMD);
    delay(5);
    t = read16(TEMPDATA);
    *temperature = t;
  #endif
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void IMU::Barometer::readRawPressure(int32_t *pressure)
{
  #if BMP085_USE_DATASHEET_VALS
    *pressure = 23843;
  #else
    uint8_t  p8;
    uint16_t p16;
    int32_t  p32;

    writeCommand(CONTROL, READPRESSURECMD + (BMPMode << 6));
    switch(BMPMode)
    {
      case ULTRALOWPOWER:
        delay(5);
        break;
      case STANDARD:
        delay(8);
        break;
      case HIGHRES:
        delay(14);
        break;
      case ULTRAHIGHRES:
      default:
        delay(26);
        break;
    }

    p16 = read16(PRESSUREDATA);
    p32 = (uint32_t)p16 << 8;
    p8 = read8(PRESSUREDATA+2);
    p32 += p8;
    p32 >>= (8 - BMPMode);

    *pressure = p32;
  #endif
}

/**************************************************************************/
/*!
    @brief  Compute B5 coefficient used in temperature & pressure calcs.
*/
/**************************************************************************/
int32_t IMU::Barometer::computeB5(int32_t ut) {
  int32_t X1 = (ut - (int32_t)BMPcoeffs.ac6) * ((int32_t)BMPcoeffs.ac5) >> 15;
  int32_t X2 = ((int32_t)BMPcoeffs.mc << 11) / (X1+(int32_t)BMPcoeffs.md);
  return X1 + X2;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
inline float IMU::Barometer::getPressurekPa()
{
  int32_t  ut = 0, up = 0, compp = 0;
  int32_t  x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;

  /* Get the raw pressure and temperature values */
  readRawTemperature(&ut);
  readRawPressure(&up);

  /* Temperature compensation */
  b5 = computeB5(ut);

  /* Pressure compensation */
  b6 = b5 - 4000;
  x1 = (BMPcoeffs.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (BMPcoeffs.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) BMPcoeffs.ac1) * 4 + x3) << BMPMode) + 2) >> 2;
  x1 = (BMPcoeffs.ac3 * b6) >> 13;
  x2 = (BMPcoeffs.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (BMPcoeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> BMPMode));

  if (b7 < 0x80000000)
  {
    p = (b7 << 1) / b4;
  }
  else
  {
    p = (b7 / b4) << 1;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  compp = p + ((x1 + x2 + 3791) >> 4);

  /* Assign compensated pressure value */
  return compp;
}

/***************************************************************************
 CONSTRUCTOR (Useless?) - (NOPE!)
 ***************************************************************************/

IMU::Barometer::Barometer(){
    begun = false;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool IMU::Barometer::begin(BMPModes mode)
{
  // Enable I2C
  Wire.begin();

  /* Mode boundary check */
    mode = ULTRAHIGHRES;

  /* Make sure we have the right device */
  uint8_t id = read8(CHIPID);
  if(id != 0x55)
  {
    return false;
  }

  /* Set the mode indicator */
  BMPMode = mode;

  /* Coefficients need to be read once */
  readCoefficients();

  return begun = true;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).
    @param  seaLevel      Sea-level pressure in hPa
*/
/**************************************************************************/
float IMU::Barometer::getAltitude(float seaLevel)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return 44330.0 * (1.0 - pow((getPressurekPa() / 100.0F) / seaLevel, 0.1903))*3.28084;
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).
    @param  altitude      Altitude in meters
*/
/**************************************************************************/
float IMU::Barometer::getSeaLevel(float altitude)
{
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return (getPressurekPa() / 100.0F) / pow(1.0 - (altitude/44330.0), 5.255);	//adjusted to return feet since that's what we use
}




};
