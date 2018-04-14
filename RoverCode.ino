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

#include "src/core/sensors/Arducam.h"
#include <Servo.h>

#define CAMERA_SERVO_PIN 6
#define CAMERA_SERVO_STOPPED 90
#define CAMERA_SERVO_RUNNING 100
#define SERVO_DELAY_TIME 1000

Servo cam_servo;

int state = 0;

void setup(){
    Serial.begin(115200);
    cam_sd_init();
}
void loop(){
  switch(state) {
    case 0:
      /*
      if (altimeter reached)
        state = 1;
        delay till landed
      */
     
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
      delay(5000);
      state = 1;
      break;
    default:
      break;
  }
}


