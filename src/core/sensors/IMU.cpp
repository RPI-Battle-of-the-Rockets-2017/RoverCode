#include "IMU.h"

namespace Rover {

IMU::IMU(){
    accelerometer = new Accelerometer();
    magnetometer = new Magnetometer();
}

IMU::~IMU(){
    delete accelerometer;
    delete magnetometer;
}

}
