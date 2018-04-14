#include "IMU.h"

namespace Rover {

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void IMU::write8(byte address, byte reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte IMU::read8(byte address, byte reg)
{
    byte value;

    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t IMU::read16(byte address, byte reg)
{
    uint16_t value;

    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)2);
    value = (Wire.read() << 8) | Wire.read();
    Wire.endTransmission();

    return value;
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
int16_t IMU::readS16(byte address, byte reg)
{
  uint16_t i = read16(address, reg);
  return (int16_t)i;
}

IMU::IMU(){
    position = 0;
	velocity = 0;
	acceleration = 0;
	prevVelocity = 0;
	prevAcceleration = 0;

    robotRot.roll = 0;
    robotRot.pitch = 0;
    robotRot.heading = 0;
	
	robotRotRates.roll = 0;
    robotRotRates.pitch = 0;
    robotRotRates.heading = 0;
	
	robotPrevRotRates.roll = 0;
    robotPrevRotRates.pitch = 0;
    robotPrevRotRates.heading = 0;
	
	sensor_counter = 0;
	
	accel_x_offset = 0;
	accel_y_offset = 0;
	accel_z_offset = 0;
	gyro_x_offset = 0;
	gyro_y_offset = 0;
	gyro_z_offset = 0;

    accelerometer = new Accelerometer();
    magnetometer = new Magnetometer();
    gyroscope = new Gyroscope();
    barometer = new Barometer();
}

IMU::~IMU(){
    delete accelerometer;
    delete magnetometer;
    delete gyroscope;
    delete barometer;
}

//Attempt to begin everything that is not yet active.
//returns true if everything is started, false otherwise.
bool IMU::begin(){
    bool success = true;
    if (!accelerometer->active()) success = (accelerometer->begin()) ? success : false;
    if (!magnetometer->active()) success = (magnetometer->begin()) ? success : false;
    if (!gyroscope->active()) success = (gyroscope->begin()) ? success : false;
    if (!barometer->active()) success = (barometer->begin()) ? success : false;
    return success;
}

bool IMU::setSleepSettings(){
	if(!accelerometer->setSleepSettings()||!magnetometer->setSleepSettings()||!gyroscope->setSleepSettings())
		return false;
	return true;
}

bool IMU::setNormalSettings(){
	if(!accelerometer->setNormalSettings()||!magnetometer->setNormalSettings()||!gyroscope->setNormalSettings())
		return false;
	return true;
}

void IMU::readAccelerometer(){
	accelerometer->getEvent(vec);
	rawAccel.x = vec.x*3.25786+.447418;
	rawAccel.y = vec.y*3.15389+.395669;
	rawAccel.z = vec.z*3.28799+.44188;
}

void IMU::readGyroscope(){
	gyroscope->getEvent(vec);
	rawGyro.pitch = vec.x+.93508;
	rawGyro.roll = vec.y+2.94101;
	rawGyro.heading = (vec.z-.32851)*1.242;
}


void IMU::updateOrientation(){
	if(sensor_counter>=1){
		sensor_counter=0;
		readAccelerometer();
		readGyroscope();
		prevAcceleration = acceleration;
		prevVelocity = velocity;
		robotPrevRotRates.heading = robotRotRates.heading;
		filter_sum-=filter[shifter];
		filter[shifter] = (rawAccel.y-accel_y_offset)/cos(robotRot.pitch);
		filter_sum+=filter[shifter];
		shifter = (shifter + 1)%sample_size;
		acceleration = filter_sum/sample_size;
		velocity = prevVelocity+prevAcceleration/320.0f+(acceleration-prevAcceleration)/640.0f;
		position += prevVelocity/320.0f+(velocity-prevVelocity)/640.0f;
		robotRotRates.heading = (rawGyro.heading-gyro_z_offset)/cos(robotRot.pitch);
		robotRot.heading += robotPrevRotRates.heading/320.0f + (robotRotRates.heading - robotPrevRotRates.heading)/640.0f;
		if(robotRot.heading>2*PI)
			robotRot.heading-=2*PI;
		if(robotRot.heading<0)
			robotRot.heading+=2*PI;
	}
}

void IMU::resetRates(){
	velocity=0;
	acceleration=0;
	prevVelocity=0;
	prevAcceleration=0;
	robotRotRates.roll=0;
	robotRotRates.pitch=0;
	robotRotRates.heading=0;
	robotPrevRotRates.roll=0;
	robotPrevRotRates.pitch=0;
	robotPrevRotRates.heading=0;
	float sumAccelX=0;
	float sumAccelY=0;
	float sumAccelZ=0;
	float sumGyroZ=0;
	shifter = 0;
	filter_sum = 0;
	for(int i=0; i<sample_size; i++)
		filter[i]=0;
	delay(1000);
	for(int i=0; i<100; i++){
		readAccelerometer();
		readGyroscope();
		sumAccelX+=rawAccel.x;
		sumAccelY+=rawAccel.y;
		sumAccelZ+=rawAccel.z;
		sumGyroZ+=rawGyro.heading;
		delay(10);
	}
	robotRot.pitch = asin(-1*sumAccelY/sqrt(pow(sumAccelX,2)+pow(sumAccelY,2)+pow(sumAccelZ,2)));
	accel_y_offset = sumAccelY/100;
	gyro_z_offset = sumGyroZ/100;
}

void IMU::resetOrientation(){
	position=0;
	robotRot.heading=0;
}

}
