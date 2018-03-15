#ifndef ROVER_IMU_ACCELEROMETER_H
#define ROVER_IMU_ACCELEROMETER_H

#include "Arduino.h"
#include <Wire.h>

#include "../IMU.h"

/**
 * Accelerometer specific functions and whatnot are defined in here
 */

namespace Rover {

class IMU::Accelerometer {
public:
    typedef struct {
      int16_t x;
      int16_t y;
      int16_t z;
    } AccelData;
    //Constructor
    Accelerometer();

    bool begin();
    //Sleep settings for launch
    bool setSleepSettings();
    //Normal settings for after launch
    bool setNormalSettings();
    bool getEvent(SensorVec&);

    AccelData raw;

private:
    //Register Values
    enum AccelRegisters {
                                      // DEFAULT    TYPE
        CTRL_REG1_A         = 0x20,   // 00000111   rw
        CTRL_REG2_A         = 0x21,   // 00000000   rw
        CTRL_REG3_A         = 0x22,   // 00000000   rw
        CTRL_REG4_A         = 0x23,   // 00000000   rw
        CTRL_REG5_A         = 0x24,   // 00000000   rw
        CTRL_REG6_A         = 0x25,   // 00000000   rw
        REFERENCE_A         = 0x26,   // 00000000   r
        STATUS_REG_A        = 0x27,   // 00000000   r
        OUT_X_L_A           = 0x28,
        OUT_X_H_A           = 0x29,
        OUT_Y_L_A           = 0x2A,
        OUT_Y_H_A           = 0x2B,
        OUT_Z_L_A           = 0x2C,
        OUT_Z_H_A           = 0x2D,
        FIFO_CTRL_REG_A     = 0x2E,
        FIFO_SRC_REG_A      = 0x2F,
        INT1_CFG_A          = 0x30,
        INT1_SOURCE_A       = 0x31,
        INT1_THS_A          = 0x32,
        INT1_DURATION_A     = 0x33,
        INT2_CFG_A          = 0x34,
        INT2_SOURCE_A       = 0x35,
        INT2_THS_A          = 0x36,
        INT2_DURATION_A     = 0x37,
        CLICK_CFG_A         = 0x38,
        CLICK_SRC_A         = 0x39,
        CLICK_THS_A         = 0x3A,
        TIME_LIMIT_A        = 0x3B,
        TIME_LATENCY_A      = 0x3C,
        TIME_WINDOW_A       = 0x3D
    };

    float accel_LSB;

    inline void write8(byte reg, byte value);
    inline byte read8(byte reg);
    void read();
};

}

#endif // ROVER_IMU_ACCELEROMETER_H
