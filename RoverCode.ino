#ifdef OPTIMIZE
#pragma GCC optimize ("-O3")
#endif // OPTIMIZE

#include <Servo.h>

#include "src/core/components/NichromeCutter.h"
#include "src/core/sensors/Camera.h"
#include "src/core/sensors/IMU.h"
#include "src/actions/Drive.h"
#include <avr/sleep.h>


//Rover::Camera cam(&Serial1);

Rover::IMU imu;
Rover::Drive drive(3, 5);

unsigned char state = 0;

unsigned char accel_gyro_counter = 0;
unsigned char magnetometer_counter = 0;
unsigned int override_counter = 0;

byte adcsra_save=0;
float slp;
float altitude;
float landing_check_array [5];

float ground_altitude = 0/3.28084; //get a real value for this


void setup() {
  // put your setup code here, to run once:
  unsigned char test = 0; //DO NOT LEAVE IN CODE, TESTING PURPOSES ONLY
  adcsra_save=ADCSRA; //save ADC settings
  Serial.begin(38400);
  drive.attach();

  imu.begin();
  while(!imu.setSleepSettings());
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(2, HIGH);
  slp = imu.barometer->getSeaLevel(ground_altitude);
  while(state==0){  //pre-launch state
    Serial.println("0");
    test++; //SERIOUSLY, TAKE THIS OUT
    ADCSRA = adcsra_save;  //reset ADC
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
  while(state==1){  //ascending state
    Serial.println("1");
    altitude = imu.barometer->getAltitude(slp);
    if(altitude-ground_altitude >= 800||override_counter>=13600)//above 800ft. or override
      state++;
  }
  while(state==2){  //descending state
    Serial.println("2");
    altitude = imu.barometer->getAltitude(slp);
    if(altitude-ground_altitude <= 400||override_counter>=13600)// below 400ft. or override
      state++;
  }
  Serial.println("Nichrome on");  //replace these 3 lines with real nichrome code
  delay(1000);
  Serial.println("Nichrome off");
  for(int i=0; i<5; i++){
    altitude = imu.barometer->getAltitude(slp);
    landing_check_array[i] = altitude; //initialize array
    delay(974);
  }
  while(state==3){
    landing_check();  //wait for landing detection
  }
  Serial.println("4");
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
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

void timer_init(){
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;

  OCR3B = 40000;            // compare match register 16MHz/400Hz
  TCCR3B |= 0x08;   // CTC mode
  TCCR3B |= 0x01;    // 1 prescaler 
  TIMSK3 |= 0x04;  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER3_COMPB_vect)          // timer compare interrupt service routine
{
  accel_gyro_counter++;
  magnetometer_counter++;
  override_counter++;
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
  Serial.println("3");
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

