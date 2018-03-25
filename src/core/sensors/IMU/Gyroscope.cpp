#include "Arduino.h"

#include <Wire.h>
//#include <limits.h>

#include "../IMU.h"
#include "Gyroscope.h"

//I2C Address
#define L3GD20_ADDRESS           (0x6B)        // 1101011
#define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
#define L3GD20_ID                (0xD4)
#define L3GD20H_ID               (0xD7)
// Sesitivity values from the mechanical characteristics in the datasheet.
#define GYRO_SENSITIVITY_250DPS  (0.00875F)
#define GYRO_SENSITIVITY_500DPS  (0.0175F)
#define GYRO_SENSITIVITY_2000DPS (0.070F)

/**< Degrees/s to rad/s multiplier */
#define SENSORS_DPS_TO_RADS      (0.017453293F)

namespace Rover {

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void IMU::Gyroscope::write8(byte reg, byte value){
    IMU::write8(L3GD20_ADDRESS, reg, value);
}
byte IMU::Gyroscope::read8(byte reg){
    return IMU::read8(L3GD20_ADDRESS, reg);
}

IMU::Gyroscope::Gyroscope() {
    autoRangeEnabled = false;
    begun = false;

    // Clear the raw gyro data
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
bool IMU::Gyroscope::begin(GyroRange rng){
  /* Enable I2C */
  Wire.begin();

  /* Set the range the an appropriate value */
  range = rng;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = read8(WHO_AM_I);
  //Serial.println(id, HEX);
  if ((id != L3GD20_ID) && (id != L3GD20H_ID))
  {
    return false;
  }

  /* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Reset then switch to normal mode and enable all three channels */
  write8(CTRL_REG1, 0x00);
  write8(CTRL_REG1, 0x0F);
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Adjust resolution if requested */
  switch(range)
  {
    case RANGE_250DPS:
      write8(CTRL_REG4, 0x00);
      break;
    case RANGE_500DPS:
      write8(CTRL_REG4, 0x10);
      break;
    case RANGE_2000DPS:
      write8(CTRL_REG4, 0x20);
      break;
  }
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  return begun = true;
}

/**************************************************************************/
/*!
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void IMU::Gyroscope::enableAutoRange(bool enabled)
{
    autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool IMU::Gyroscope::getEvent(SensorVec& event)
{
  bool readingValid = false;

  /* Clear the raw data placeholder */
  raw.x = 0;
  raw.y = 0;
  raw.z = 0;

  while(!readingValid)
  {
    event.timestamp = millis();

    /* Read 6 bytes from the sensor */
    Wire.beginTransmission((byte)L3GD20_ADDRESS);
    Wire.write(OUT_X_L | 0x80);
    if (Wire.endTransmission() != 0) {
        // Error. Retry.
        continue;
    }
    Wire.requestFrom((byte)L3GD20_ADDRESS, (byte)6);

    uint8_t xlo = Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t zhi = Wire.read();

    /* Shift values to create properly formed integer (low byte first) */
    event.x = (int16_t)(xlo | (xhi << 8));
    event.y = (int16_t)(ylo | (yhi << 8));
    event.z = (int16_t)(zlo | (zhi << 8));

    /* Assign raw values in case someone needs them */
    raw.x = (int16_t)(xlo | (xhi << 8));
    raw.y = (int16_t)(ylo | (yhi << 8));
    raw.z = (int16_t)(zlo | (zhi << 8));

    /* Make sure the sensor isn't saturating if auto-ranging is enabled */
    if (!autoRangeEnabled)
    {
      readingValid = true;
    }
    else
    {
      /* Check if the sensor is saturating or not */
      if ( (event.x >= 32760) | (event.x <= -32760) |
           (event.y >= 32760) | (event.y <= -32760) |
           (event.z >= 32760) | (event.z <= -32760) )
      {
        /* Saturating .... increase the range if we can */
        switch(range)
        {
          case RANGE_500DPS:
            /* Push the range up to 2000dps */
            range = RANGE_2000DPS;
            write8(CTRL_REG1, 0x00);
            write8(CTRL_REG1, 0x0F);
            write8(CTRL_REG4, 0x20);
            write8(CTRL_REG5, 0x80);
            readingValid = false;
            // Serial.println("Changing range to 2000DPS");
            break;
          case RANGE_250DPS:
            /* Push the range up to 500dps */
            range = RANGE_500DPS;
            write8(CTRL_REG1, 0x00);
            write8(CTRL_REG1, 0x0F);
            write8(CTRL_REG4, 0x10);
            write8(CTRL_REG5, 0x80);
            readingValid = false;
            // Serial.println("Changing range to 500DPS");
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

  /* Compensate values depending on the resolution */
  switch(range)
  {
    case RANGE_250DPS:
      event.x *= GYRO_SENSITIVITY_250DPS;
      event.y *= GYRO_SENSITIVITY_250DPS;
      event.z *= GYRO_SENSITIVITY_250DPS;
      break;
    case RANGE_500DPS:
      event.x *= GYRO_SENSITIVITY_500DPS;
      event.y *= GYRO_SENSITIVITY_500DPS;
      event.z *= GYRO_SENSITIVITY_500DPS;
      break;
    case RANGE_2000DPS:
      event.x *= GYRO_SENSITIVITY_2000DPS;
      event.y *= GYRO_SENSITIVITY_2000DPS;
      event.z *= GYRO_SENSITIVITY_2000DPS;
      break;
  }

  /* Convert values to rad/s */
  event.x *= SENSORS_DPS_TO_RADS;
  event.y *= SENSORS_DPS_TO_RADS;
  event.z *= SENSORS_DPS_TO_RADS;

  return true;
}

}
