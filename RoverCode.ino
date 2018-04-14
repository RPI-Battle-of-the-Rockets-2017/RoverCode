// ArduCAM demo (C)2017 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with a supported camera modules, and can run on any Arduino platform.
//
// This demo was made for Omnivision 2MP/5MP sensor.
// It will run the ArduCAM 2MP/5MP as a real 2MP/5MP digital camera, provide both JPEG capture.
// The demo sketch will do the following tasks:
// 1. Set the sensor to JPEG mode.
// 2. Capture and buffer the image to FIFO every 5 seconds 
// 3. Store the image to Micro SD/TF card with JPEG format in sequential.
// 4. Resolution can be changed by myCAM.set_JPEG_size() function.
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM 2MP/5MP shield
// and use Arduino IDE 1.6.8 compiler or above

#include "src/core/sensors/ArduCAM.h"
#include "src/core/components/NichromeCutter.h"
#include <Servo.h>

#define CAMERA_SERVO_PIN 9
#define STRATOLOGGER_PIN 7
#define CAMERA_SERVO_STOPPED 93
#define CAMERA_SERVO_RUNNING 100
#define SERVO_DELAY_TIME 500
#define MAX_PICTURES 25

Servo cam_servo;
Rover::NichromeCutter nc(10);

int state = 0;
int count = 0;

void setup(){
    Serial.begin(115200);
    cam_sd_init();
}
void loop(){
  switch(state) {
    case 0:
      while(digitalRead(7)==HIGH);
      state = 1;
      delay(30*1000);
      nc.activate(5000);
      break;
    case 1:
      cam_servo.attach(CAMERA_SERVO_PIN);
      cam_servo.write(CAMERA_SERVO_RUNNING);
      delay(SERVO_DELAY_TIME);
      cam_servo.write(CAMERA_SERVO_STOPPED);
      cam_servo.detach();
      state = 2;
      break;
    case 2:
      myCAMSaveToSDFile();
      count++;
      delay(5000);
      state = 1;
      if(count>=MAX_PICTURES)
        state = 3;
      break;
    case 3:
      while(1);
    default:
      break;
  }
}


