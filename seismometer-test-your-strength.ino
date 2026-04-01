


/* TODO
handle probablities in ints
do drawing in a series of slices spread over time
maybe do accelerometer interrupts?
choose best axis for accelerometer

*/

/**************************************************************************/
/*!
    @file     Adafruit_MMA8451.h
  made from the Adafruit example for the Adafruit MMA8451 Accel breakout board
    ----> https://www.adafruit.com/products/2019

Runs on an Feather ESP32-S3 2MB PSRAM
Select esp32/Feather ESP32-S3 2MB PSRAM

!*/

#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


// #define GRAPH_ACCELEROMETER

hw_timer_t *accelerometerTimer = NULL;

Adafruit_MMA8451 mma = Adafruit_MMA8451();

double expectedNoise = 2;

void setupLEDPanels();
void loopLEDPanels();

void setup(void) {
   Serial.begin(115200);
   delay(5000);
  Serial.println("Begin...");
  setupLEDPanels();
  setupAccelerometer();
}

void setupAccelerometer()
{
  

  if (! mma.begin()) {
    Serial.println("Couldnt start accelerometer");
    delay(1000);
    while (1);
  }
  Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);
  // lower datarate means lower noise
  // 50hz gives a noise of 2
  // 100hz gives 3
  // 800hz gives a noise of 8
  // our loop, with printing in, is running at 500hz
  mma.setDataRate(MMA8451_DATARATE_100_HZ); //MMA8451_DATARATE_800_HZ);
  expectedNoise = 3;

  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
  
}


double mx = 0;
double my = 0;
double mz = 0;
double mn = 0;

bool first = true;

bool was = false;
int pulse = 0;
long lastMicros = 0;

int loops = 0;
void loop() {
  loops++;
  if( loops%100 == 0)
  {
    //Serial.print("loop ");
    //Serial.println(loops);
  }
  loopLEDPanels();
  loopAccelerometer();
  delay(10);
}

void loopAccelerometer()
{
  readAccelerometer();
}

void readAccelerometer()
{
  // can't do i2c inside an interrupt handler
  double a1 = 0.99;
  double a2 = 1.0-a1;

  long m = micros();
  long interval = m-lastMicros; // typically 2000
  lastMicros = m;

  // Read the 'raw' data in 14-bit counts
  mma.read();
  if( first)
  {
    first = false;
    mx = mma.x;
    my = mma.y;
    mz = mma.z;
  }
  else
  {
    

    mx = a1*mx+a2*mma.x;
    my = a1*my+a2*mma.y;
    mz = a1*mz+a2*mma.z;
  }
  
  double t = expectedNoise*4.5;

  double noise = (fabs(mx-mma.x) + fabs(my-mma.y) + fabs(mz-mma.z))/3.0;

  mn = a1*mn+a2*noise;

  if( fabs(mx-mma.x)>t || fabs(my-mma.y)>t || fabs(mz-mma.z)>t )
  {
    if( ! was)
    {
      pulse ++;
      if( pulse>50)
        pulse = 0;

      #ifdef GRAPH_ACCELEROMETER
      // make separate pulses visible in the graph
      for( int i = 0; i< 20; i ++ )
      {
        Serial.print(pulse);
        Serial.println(",0,0,0,0");
      }
      #endif
    }
    was = true;
    /*
    Serial.print(mx);
    Serial.print(", ");
    Serial.print(my);
    Serial.print(", ");
    Serial.print(mz);

    Serial.print(",      ");
    */

#ifdef GRAPH_ACCELEROMETER
    //Serial.print(interval);
    //Serial.print(", ");
    Serial.print(pulse);
    Serial.print(", ");
    //Serial.print(interval);
    //Serial.print(", ");
    Serial.print(mn);
    Serial.print(", ");

    Serial.print(mx-mma.x);
    Serial.print(", ");
    Serial.print(my-mma.y);
    Serial.print(", ");
    Serial.println(mz-mma.z);
  #endif
  }
  else
  {
    was = false;
  }

}
