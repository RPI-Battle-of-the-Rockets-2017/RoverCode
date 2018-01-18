#pragma GCC optimize ("-O3")

#include "src/core/components/NichromeCutter.h"
#include "src/core/sensors/Camera.h"
#include "src/core/sensors/IMU.h"
#include "src/core/utilities/Standby.h"
#include "src/actions/Drive.h"
#include "src/core/utilities/TaskScheduler.h"

Rover::Camera camera = Rover::Camera(&Serial);

Rover::IMU imu;

void setup() {
  // put your setup code here, to run once:
  imu.accelerometer->begin();
  imu.magnetometer->begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
