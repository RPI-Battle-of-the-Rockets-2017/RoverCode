#ifndef ROVER_BAROMETER_H
#define ROVER_BAROMETER_H

#include "Arduino.h"
#include <Wire.h>

#include "../IMU.h"

/**
 * Barometer specific functions and whatnot are defined in here
 */

namespace Rover {

class IMU::Barometer {
private:
    //Register Values
    enum BMPRegisters {
        REG_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
        REG_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
        REG_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
        REG_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
        REG_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
        REG_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
        REG_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
        REG_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
        REG_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
        REG_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
        REG_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
        REG_CHIPID             = 0xD0,
        REG_VERSION            = 0xD1,
        REG_SOFTRESET          = 0xE0,
        REG_CONTROL            = 0xF4,
        REG_TEMPDATA           = 0xF6,
        REG_PRESSUREDATA       = 0xF6,
        REG_READTEMPCMD        = 0x2E,
        REG_READPRESSURECMD    = 0x34
    };

    //Calibration structure
    typedef struct
    {
        int16_t  ac1;
        int16_t  ac2;
        int16_t  ac3;
        uint16_t ac4;
        uint16_t ac5;
        uint16_t ac6;
        int16_t  b1;
        int16_t  b2;
        int16_t  mb;
        int16_t  mc;
        int16_t  md;
    } BMPCalibData;

    int32_t computeB5(int32_t ut);

    BMPCalibData BMPcoeffs;   // Last read accelerometer data will be available here
    uint8_t BMPMode;

    inline void writeCommand(byte reg, byte value);
    inline byte read8(byte reg);
    inline uint16_t read16(byte reg);
    inline int16_t readS16(byte reg);
    void readCoefficients();
    void readRawTemperature(int32_t *temperature);
    void readRawPressure(int32_t *pressure);
    inline float getPressurekPa();
public:
    //Mode Settings
    enum BMPModes {
        MODE_ULTRALOWPOWER          = 0,
        MODE_STANDARD               = 1,
        MODE_HIGHRES                = 2,
        MODE_ULTRAHIGHRES           = 3
    };


    //Adafruit_BMP085_Unified();

    bool begin(BMPModes mode = MODE_ULTRAHIGHRES);
    float getAltitude(float seaLevel);
    float getSeaLevel(float altitude);
};

}

#endif // ROVER_BAROMETER_H
