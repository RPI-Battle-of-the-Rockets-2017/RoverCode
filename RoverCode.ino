#define OPTIMIZE
#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

#include "src/core/components/NichromeCutter.h"
#include "src/core/sensors/Camera.h"
#include "src/core/sensors/IMU.h"
#include "src/core/utilities/Standby.h"
#include "src/actions/Drive.h"
//#include "src/core/utilities/TaskScheduler.h"

//Rover::Camera camera = Rover::Camera(&Serial);

Rover::IMU imu;

Rover::IMU::SensorVec vector;

void setup() {
  // put your setup code here, to run once:
  imu.begin();
  //imu.magnetometer->begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.accelerometer->getEvent(vector);
  Serial.print("x: ");
  Serial.print(vector.x);
  Serial.print(", y: ");
  Serial.print(vector.y);
  Serial.print(", z: ");
  Serial.println(vector.z);
  imu.gyroscope->getEvent(vector);
  Serial.print("x: ");
  Serial.print(vector.x);
  Serial.print(", y: ");
  Serial.print(vector.y);
  Serial.print(", z: ");
  Serial.println(vector.z);
  imu.magnetometer->getEvent(vector);
  Serial.print("x: ");
  Serial.print(vector.x);
  Serial.print(", y: ");
  Serial.print(vector.y);
  Serial.print(", z: ");
  Serial.println(vector.z);
  float altitude = imu.barometer->getAltitude(1016.2);
  Serial.print("altitude: ");
  Serial.println(altitude);

  delay(1000);
}
