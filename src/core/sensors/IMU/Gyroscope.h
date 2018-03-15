#ifndef ROVER_IMU_GYROSCOPE_H
#define ROVER_IMU_GYROSCOPE_H

#include "Arduino.h"
#include <Wire.h>

#include "../IMU.h"

/**
 * Gyroscope specific functions and whatnot are defined in here
 */

namespace Rover {

class IMU::Gyroscope {
public:
    //Optional Speed Settings
    enum GyroRange{
      RANGE_250DPS  = 250,
      RANGE_500DPS  = 500,
      RANGE_2000DPS = 2000
    };

    //Raw Data structure
    typedef struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } GyroData;

    Gyroscope();

    bool begin(GyroRange rng = RANGE_250DPS);
    void enableAutoRange(bool enabled);
    bool getEvent(SensorVec&);
    //If needed, implement max_value and min_value getters
    //Refer to adafruit library to see relevant conversions.

    GyroData raw; /* Raw values from last sensor read */

private:
    //Register Values
    enum GyroRegisters {
                                                  // DEFAULT    TYPE
      WHO_AM_I            = 0x0F,   // 11010100   r
      CTRL_REG1           = 0x20,   // 00000111   rw
      CTRL_REG2           = 0x21,   // 00000000   rw
      CTRL_REG3           = 0x22,   // 00000000   rw
      CTRL_REG4           = 0x23,   // 00000000   rw
      CTRL_REG5           = 0x24,   // 00000000   rw
      REFERENCE           = 0x25,   // 00000000   rw
      OUT_TEMP            = 0x26,   //            r
      STATUS_REG          = 0x27,   //            r
      OUT_X_L             = 0x28,   //            r
      OUT_X_H             = 0x29,   //            r
      OUT_Y_L             = 0x2A,   //            r
      OUT_Y_H             = 0x2B,   //            r
      OUT_Z_L             = 0x2C,   //            r
      OUT_Z_H             = 0x2D,   //            r
      FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
      FIFO_SRC_REG        = 0x2F,   //            r
      INT1_CFG            = 0x30,   // 00000000   rw
      INT1_SRC            = 0x31,   //            r
      TSH_XH              = 0x32,   // 00000000   rw
      TSH_XL              = 0x33,   // 00000000   rw
      TSH_YH              = 0x34,   // 00000000   rw
      TSH_YL              = 0x35,   // 00000000   rw
      TSH_ZH              = 0x36,   // 00000000   rw
      TSH_ZL              = 0x37,   // 00000000   rw
      INT1_DURATION       = 0x38    // 00000000   rw
    };

    GyroRange range;
    bool autoRangeEnabled;

    inline void write8(byte reg, byte value);
    inline byte read8(byte reg);
};

};

#endif
