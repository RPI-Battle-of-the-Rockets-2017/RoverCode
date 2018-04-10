#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

#include <Servo.h>

#include "src/core/components/NichromeCutter.h"
#include "src/core/sensors/Camera.h"
#include "src/core/sensors/IMU.h"
#include "src/core/sensors/Adafruit_GPS.h"
#include "src/actions/Drive.h"
#include "src/core/utilities/Telemetry.h"
#include "src/core/components/SDUtils.h"
#include <avr/sleep.h>

#define GPSSerial Serial2


Rover::Camera cam(&Serial1);
Rover::Telemetry tele(&Serial3, &Serial);

Rover::IMU imu;
Rover::Drive drive(3, 5, &imu, true);
Rover::NichromeCutter nc1(5);
Rover::NichromeCutter nc2(6);
Rover::NichromeCutter nc3(7);
Rover::NichromeCutter nc4(4);
Adafruit_GPS GPS(&GPSSerial);


unsigned char state = 0;

unsigned int override_counter = 0;
unsigned int tele_counter=0;

byte adcsra_save=0;
float slp;
float altitude;
float landing_check_array [5];
long lat;
long lon;
char c;

float ground_altitude = 273; //get a real value for this
int speed = 50; //decided from testing
int nichrome_time = 5000;

void setup() {
  // put your setup code here, to run once:
  unsigned char test = 0; //DO NOT LEAVE IN CODE, TESTING PURPOSES ONLY
  adcsra_save=ADCSRA; //save ADC settings
  Serial.begin(38400);
  nc1.attach();
  nc2.attach();
  nc3.attach();
  nc4.attach();

  tele.pack_ser.setPacketHandler([](const uint8_t *buffer, size_t size) {
    tele.processTelem(buffer, size);
  });

  cam.begin();
  cam.setImageSize(VC0706_640x480);
  
  Rover::SDUtils sd;
  cam.takePicture();
  sd.writeImage(cam, tele);

  /*GPS.begin(9600);
  GPS.sendCommand("$PGCMD,33,0*6D"); //So antenna doesn't act up
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate*/

  imu.begin();
  while(!imu.setSleepSettings());
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(2, INPUT);
  slp = imu.barometer->getSeaLevel(ground_altitude/3.28084);
  while(state==0){  //pre-launch state
    Serial.println("0");
    test++; //SERIOUSLY, TAKE THIS OUT
    ADCSRA = adcsra_save;  //reset ADC
    while(digitalRead(2) == LOW);
    altitude = imu.barometer->getAltitude(slp);
    delay(974);
    if(imu.barometer->getAltitude(slp) - altitude >= 100 || test ==3) //check for 100 ft. difference in 1 second
      state++;
    else
      sleep(); 
  }
  while(!imu.setNormalSettings());
  override_counter = 0;
  timer_init();
  Serial.println("1");
  while(state==1){  //ascending state
    altitude = imu.barometer->getAltitude(slp);
    if(altitude-ground_altitude >= 800||override_counter>=10880)//above 800ft. or override
      state++;
    if(tele_counter>=320){
      tele_counter=0;
      tele.sendBaroHeight(altitude);
    }
    tele.update();
  }
  Serial.println("2");
  while(state==2){  //descending state
    altitude = imu.barometer->getAltitude(slp);
    if(altitude-ground_altitude <= 400||override_counter>=10880)// below 400ft. or override
      state++;
    if(tele_counter>=320){
      tele_counter=0;
      tele.sendBaroHeight(altitude);
    }
    tele.update();
  }
  nc1.activate(nichrome_time);
  Serial.println("3");
  for(int i=0; i<5; i++){
    altitude = imu.barometer->getAltitude(slp);
    landing_check_array[i] = altitude; //initialize array
    delay(974);
  }
  while(state==3){
    landing_check();  //wait for landing detection
    if(tele_counter>=320){
      tele_counter=0;
      tele.sendBaroHeight(altitude);
    }
    tele.update();
  }
  drive.attach();
  drive.halt();
  Serial.println("4");
  nc2.activate(nichrome_time);
  nc3.activate(nichrome_time);
  //camera stuff
  /*Serial.begin(38400);
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

  sd.writeImage(cam);*/
  for(int i=0; i<10; i++){
    forward(1);
    readGPS();
    tele.sendGPSData(lat, lon);
  }
  nc4.activate(nichrome_time);
  turn();
  readGPS();
  tele.sendGPSData(lat, lon);
  for(int i=0; i<10; i++){
    forward(1);
    readGPS();
    tele.sendGPSData(lat, lon);
  }
}

void loop() {
  readGPS();
  tele.sendGPSData(lat, lon);
  tele.update();
}

void timer_init(){
noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  TCCR3B |= 0x01;   // Overflow mode
  TIMSK3 |= 0x01;  // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER3_OVF_vect)          // timer compare interrupt service routine
{
  imu.sensor_counter++;
  override_counter++;
  TCNT3 = 15535;  //set timer for 320Hz
}

void wake ()
{
  // cancel sleep as a precaution
  sleep_disable();
  // precautionary while we do other stuff
  detachInterrupt (digitalPinToInterrupt (2));
}

void sleep(){
  // disable ADC
  adcsra_save=ADCSRA;
  ADCSRA = 0;  
     
  set_sleep_mode (SLEEP_MODE_PWR_SAVE); 
  sleep_enable();

  // Do not interrupt before we go to sleep, or the
  // ISR will detach interrupts and we won't wake.
  noInterrupts ();
  
  // will be called when pin D2 goes low  
  attachInterrupt (digitalPinToInterrupt (2), wake, LOW);
  EIFR = bit (INTF0);  // clear flag for interrupt 0

  
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts ();  // one cycle 
  sleep_cpu ();   // one cycle
}


void landing_check(){
  if(landing_check_array[0] - landing_check_array[4] <= 20)  //difference between current altitude and altitude 5 seconds ago
    state++;
  else{
    landing_check_array[0] = landing_check_array[1];
    landing_check_array[1] = landing_check_array[2];
    landing_check_array[2] = landing_check_array[3];
    landing_check_array[3] = landing_check_array[4];
    altitude = imu.barometer->getAltitude(slp);
    landing_check_array[4] = altitude; //update altitude
    delay(974);
  }
}
 void forward(float distance){
    imu.resetRates();
    float initial = imu.getPosition();
    while(imu.getPosition() - initial < distance){
      drive.drive(speed); //lol
      imu.updateOrientation();
      tele.update();
    }
    drive.halt();
 }

 void turn(){
    imu.resetRates();
    while(imu.getHeading()>3*PI/2||imu.getHeading()<PI/2){
      drive.pivotTurn(speed, -1);
      imu.updateOrientation();
      tele.update();
    }
    drive.halt();
    imu.resetOrientation();
 }

void readGPS(){
  while(!GPS.newNMEAreceived()) //Keep reading characters in this loop until a good NMEA sentence is received
    c=GPS.read(); //read a character from the GPS
  GPS.parse(GPS.lastNMEA());  //Once you get a good NMEA, parse it
  lat = GPS.latitude*100000;
  lon = GPS.longitude*100000;
 }
 



