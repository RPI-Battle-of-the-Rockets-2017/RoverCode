/*
#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

  #include "src/core/components/NichromeCutter.h"
  #include "src/core/components/SDUtils.h"
  #include "src/core/sensors/Camera.h"
  #include "src/core/sensors/IMU.h"
  #include "src/core/utilities/Standby.h"
  #include "src/actions/Drive.h"
  #include "src/core/utilities/TaskScheduler.h"
*/

#include <Servo.h>

#include "src/actions/Drive.h"

//Rover::Camera cam(&Serial1);
/*
Servo left;
Servo right;

//Rover::IMU imu;

#define MIN 30
#define MAX 150
#define MID 90*/

/*
  void setup() {
  // put your setup code here, to run once:
  //imu.accelerometer->begin();
  //imu.magnetometer->begin();
  Serial.begin(38400);
  if (!cam.begin()) {
    Serial.printlnln("No camera found");
    while(1){};
  }
  Serial.println("Hello world\n");
  delay(2000);
  Rover::SDUtils sd;
  cam.setImageSize(VC0706_640x480);
  if (!cam.takePicture()) {
    Serial.printlnln("Failed to take picture");
  }

  sd.writeImage(cam);
  }

  void loop() {
  // put your main code here, to run repeatedly:
  }
*/

Rover::Drive drive(3, 5);

void setup() {
  Serial.begin(38400);
  drive.attach();
  //left.attach(3);
  //right.attach(5);
}

void loop() {
  char incoming;

  if (Serial.available() > 0) {
    incoming = Serial.read();

    if (incoming == 'w') {
      //left.write(MAX);
      //right.write(MIN);
      drive.drive(100);
      while (Serial.available() == 0) {
        Serial.println(Serial.available());
        Serial.println("Driving forward");
        delay(15);
      }
    }
    else if (incoming == 'a') {
      //left.write(MIN);
      //right.write(MIN);
      drive.centerTurn(100);
      while (Serial.available() == 0) {
        Serial.println(Serial.available());
        Serial.println("Turning left");
        delay(15);
      }
    }
    else if (incoming == 's') {
      //left.write(MIN);
      //right.write(MAX);
      drive.drive(-100);
      while (Serial.available() == 0) {
        Serial.println(Serial.available());
        Serial.println("Driving backwards");
        delay(15);
      }
    }
    else if (incoming == 'd') {

      //left.write(MAX);
      //right.write(MAX);
      drive.centerTurn(-100);
      while (Serial.available() == 0) {
        Serial.println(Serial.available());
        Serial.println("Turning right");
        delay(15);
      }
    } else {
      //left.write(MID);
      //right.write(MID);
      drive.halt();
      while (Serial.available() == 0) {
        Serial.println(Serial.available());
        delay(15);
      }
    }
  }
}
