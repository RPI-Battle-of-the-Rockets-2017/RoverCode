#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

#include "src/core/sensors/IMU.h"

Rover::IMU imu;

float slp = 1015.6;
float altitude;

float sum = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
   imu.begin();
   while(!imu.setSleepSettings());
   while(!imu.setNormalSettings());
   for(int i=0; i<10000; i++)
    sum+=imu.barometer->getAltitude(slp);
   Serial.println(sum/10000.0F);
}

void loop() {
  // put your main code here, to run repeatedly:

}
