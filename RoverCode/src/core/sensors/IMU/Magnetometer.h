#ifndef ROVER_IMU_MAGNETOMETER_H
#define ROVER_IMU_MAGNETOMETER_H

#include "Arduino.h"
#include <Wire.h>

#include "../IMU.h"

/**
 * Magnetometer specific functions and whatnot are defined in here
 */

namespace Rover {

class IMU::Magnetometer {
public:
    typedef struct {
      int16_t x;
      int16_t y;
      int16_t z;
    } MagData;

    enum MagGain{
        GAIN_1_3 = 0x20,  // +/- 1.3
        GAIN_1_9 = 0x40,  // +/- 1.9
        GAIN_2_5 = 0x60,  // +/- 2.5
        GAIN_4_0 = 0x80,  // +/- 4.0
        GAIN_4_7 = 0xA0,  // +/- 4.7
        GAIN_5_6 = 0xC0,  // +/- 5.6
        GAIN_8_1 = 0xE0   // +/- 8.1
    };

    enum MagRate {
        RATE_0_7  = 0x00,  // 0.75 Hz
        RATE_1_5  = 0x01,  // 1.5 Hz
        RATE_3_0  = 0x62,  // 3.0 Hz
        RATE_7_5  = 0x03,  // 7.5 Hz
        RATE_15   = 0x04,  // 15 Hz
        RATE_30   = 0x05,  // 30 Hz
        RATE_75   = 0x06,  // 75 Hz
        RATE_200  = 0x07   // 200 Hz
    };
    //Constructor
    Magnetometer();

    bool active() const { return begun; };

    bool begin();
    void enableAutoRange(bool enable);
    void setMagGain(MagGain gain);
    void setMagRate(MagRate rate);
    bool getEvent(SensorVec&);
	bool setSleepSettings();
	bool setNormalSettings();

    MagData raw;
private:
    //Register Values
    enum MagRegisters {
        CRA_REG_M       = 0x00,
        CRB_REG_M       = 0x01,
        MR_REG_M        = 0x02,
        OUT_X_H_M       = 0x03,
        OUT_X_L_M       = 0x04,
        OUT_Z_H_M       = 0x05,
        OUT_Z_L_M       = 0x06,
        OUT_Y_H_M       = 0x07,
        OUT_Y_L_M       = 0x08,
        SR_REG_Mg       = 0x09,
        IRA_REG_M       = 0x0A,
        IRB_REG_M       = 0x0B,
        IRC_REG_M       = 0x0C,
        TEMP_OUT_H_M    = 0x31,
        TEMP_OUT_L_M    = 0x32
    };

    float _lsm303Mag_Gauss_LSB_XY;  // Varies with gain
    float _lsm303Mag_Gauss_LSB_Z;   // Varies with gain
    bool autoRangeEnabled;
    MagGain gain;
    bool begun;

    inline void write8(byte reg, byte value);
    inline byte read8(byte reg);
    void read(void);
};

}

#endif // ROVER_IMU_MAGNETOMETER_H
