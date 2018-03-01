#pragma GCC optimize ("-O3")

#include "src/core/components/NichromeCutter.h"
#include "src/core/sensors/Camera.h"
#include "src/core/sensors/IMU.h"
#include "src/core/utilities/Standby.h"
#include "src/actions/Drive.h"
//#include "src/core/utilities/TaskScheduler.h"

//Rover::Camera camera = Rover::Camera(&Serial);

Rover::IMU imu;

Rover::IMU::sensorVec vector;

void setup() {
  // put your setup code here, to run once:
  imu.accelerometer->begin();
  //imu.magnetometer->begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.accelerometer->getEvent(&vector);

  Serial.println(vector.z);
  delay(500);
}
