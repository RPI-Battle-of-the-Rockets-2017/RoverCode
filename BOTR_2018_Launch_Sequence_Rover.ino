#include <avr/sleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(12345);

float slp= 998.5;//Set to latest Sea Level Pressure measurement at nearest airport
//Theoretically we could make this more accurate if we knew the altitude of the launch site
float takeoffVelocity=5;//Set to velocity (in m/s) for takeoff detection

float altitude=0;
float pressure=0;
int launched=0;
byte adcsra_save=0;
  
void wake ()
{
  // cancel sleep as a precaution
  sleep_disable();
  // precautionary while we do other stuff
  detachInterrupt (digitalPinToInterrupt (2));
}


void setup () 
  {
    Wire.begin();
    bmp.begin();
    delay(100);
  
    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x20); //7-bit address of CTRL_REG1_A
    Wire.write(0x7F); //Sets data rate to 400Hz, enables low power mode
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x22); //7-bit address of CTRL_REG3_A
    Wire.write(0x40); //Enables AOI1 interrupt on INT1
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x23); //7-bit address of CTRL_REG4_A
    Wire.write(0x10); //Sets scale to +/- 4g
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x25); //7-bit address of CTRL_REG6_A
    Wire.write(0x02); //Sets interrupt to active low
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x30); //7-bit address of INT1_CFG_A
    Wire.write(0x2A); //Enables interrupt generation for high X OR high Y OR high Z
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x32); //7-bit address of INT1_THS_A
    Wire.write(0x50); //Sets interrupt threshold to 2.5g
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x33); //7-bit address of INT1_DURATION_A
    Wire.write(0x19); //Sets minimum duration for interrupt trigger to 100ms (5/50)
    Wire.endTransmission();
    delay(100);
  
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    digitalWrite(2, HIGH);
  
    while(launched<1){
      ADCSRA= adcsra_save;

      bmp.getPressure(&pressure);
      pressure/=100.0F;
      altitude=bmp.pressureToAltitude(slp, pressure);
      delay(500);
      bmp.getPressure(&pressure);
      pressure/=100.0F;


      if((bmp.pressureToAltitude(slp, pressure)-altitude)*2>=takeoffVelocity){
        launched=1;
        break;
      }
     
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
    Wire.beginTransmission(0x19); //7-bit address of accelerometer
    Wire.write(0x20); //7-bit address of CTRL_REG1_A
    Wire.write(0x77); //Sets data rate to 400Hz, disables low power mode
    Wire.endTransmission();

  
    //post-launch code goes here

 
  }

void loop () 
{

  }



 
