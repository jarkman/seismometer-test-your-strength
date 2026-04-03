


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
#include <CircularBuffer.hpp>


struct sample
{
  unsigned long ms; // good for 50 days
  float accel;
};

CircularBuffer<struct sample, 1000> accelBuffer; // big enough for any single hit

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
  delay(1);
}

void loopAccelerometer()
{
  readAccelerometer();
}

#define NUM_GREATEST_HITS 10
struct sample greatestHits[NUM_GREATEST_HITS];

int addGreatestHit(unsigned long ms, float accel)
{

  unsigned long now = millis();

  Serial.println("before purge---------");
  // print the array
  for(int i = 0; i <NUM_GREATEST_HITS; i ++) // biggest hits first
  {
    Serial.print("Hit ");
      Serial.print(i);
      Serial.print(" ms ");
      Serial.print(ms);
      Serial.print(" accel ");
      Serial.println( accel );
  }
  Serial.println();

  //purge old hits, move others up, fill with 0s
  for(int i = 0; i <NUM_GREATEST_HITS; i ++) 
  {
    if( greatestHits[i].ms != 0 && greatestHits[i].ms < now-60*60*1000) // an hour ago
    {
      Serial.print("Purging "); Serial.println(i);

      //for(int j = i; j<NUM_GREATEST_HITS-1; j++)
      //  greatestHits[j] = greatestHits[j+1];

      greatestHits[NUM_GREATEST_HITS-1].ms = 0;
      greatestHits[NUM_GREATEST_HITS-1].accel = 0;
    }
  }

Serial.println("after purge---------");
  // print the array
  for(int i = 0; i <NUM_GREATEST_HITS; i ++) // biggest hits first
  {
    Serial.print("Hit ");
      Serial.print(i);
      Serial.print(" ms ");
      Serial.print(ms);
      Serial.print(" accel ");
      Serial.println( accel );
  }
  Serial.println();

  for(int i = 0; i <NUM_GREATEST_HITS; i ++) // biggest hits first
  {
    Serial.print("Comparing hit ");
      Serial.print(i);
      Serial.print(" ms ");
      Serial.print(ms);
      Serial.print(" accel ");
      Serial.println( accel );

    if(accel > greatestHits[i].accel)
    {
      // move everything along
      //for(int j = NUM_GREATEST_HITS-2; j>=i; j--)
      //  greatestHits[j+1] = greatestHits[j];

      greatestHits[i].ms = ms;
      greatestHits[i].accel = accel;

      Serial.print("Inserting hit ");
      Serial.print(i);
      Serial.print(" ms ");
      Serial.print(ms);
      Serial.print(" accel ");
      Serial.println( accel );


      return i;
    }
  }

  return -1;
}

int logBurst()
{
  using index_t = decltype(accelBuffer)::index_t;
	
  unsigned long start = 0;
  if( accelBuffer.size() < 1)
    return -1;

  start = accelBuffer[0].ms;

  float maxAccel = 0.0;

  for (index_t i = 0; i < accelBuffer.size(); i++) 
  {
    if(accelBuffer[i].accel > maxAccel)
      maxAccel = accelBuffer[i].accel;

	  Serial.print( i );
    Serial.print( ", " );
    Serial.print( accelBuffer[i].ms - start );
    Serial.print( ", " );
    Serial.println( accelBuffer[i].accel );
 
  }

  int hit = addGreatestHit(accelBuffer[0].ms, maxAccel);

  if( hit > -1 )
  {
    Serial.print("Greatest hit ");
    Serial.println(hit);
  }
  Serial.println();
  Serial.println();

  return hit;
}
float vuLevel = 0;
long burstStart = 0;
bool inBurst = false;

#define USE_LOGS
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
    
    // maintain a rolling-average of the signal as a noise floor
    mx = a1*mx+a2*mma.x;
    my = a1*my+a2*mma.y;
    mz = a1*mz+a2*mma.z;
  }
  
  double t = expectedNoise*4.5; // set a threshold from the sensor noise we've seen with complete quiet

  double noise = (fabs(mx-mma.x) + fabs(my-mma.y) + fabs(mz-mma.z))/3.0;

  if(noise>t)
  {
    if( ! inBurst)
    {
      burstStart = millis();
      inBurst = true;
      accelBuffer.clear();
    }
  }
  else
  {
    if(millis()-burstStart > 200 && inBurst)
    {
      inBurst = false;
      logBurst();
    }
  }

  // keep a buffer for each burst so we can measure its size after it ends
  if(inBurst)
  {
    struct sample sample;

    sample.ms = millis();
    sample.accel = noise;
    accelBuffer.push(sample);
  }
  mn = a1*mn+a2*noise;  // maintain a rolling average of the total signal

#ifdef USE_LOGS
// bar display happens at all times, not just in bursts
  float noiseLimit = 50; // TODO - calibrate
  float logNoise;

  if(noise>expectedNoise)
    logNoise = log10((noise-expectedNoise)*10.0/noiseLimit); // range is 0 to 1
  else
    logNoise = 0;

  if(logNoise < 0)
    logNoise = 0;

  if(logNoise > 1)
    logNoise = 1;

  if( logNoise > vuLevel)
    vuLevel = logNoise;
  else
    vuLevel = vuLevel*(1.0-interval/2000000.0);

 // Serial.print("noise-expected "); Serial.print(noise-expectedNoise);
//Serial.print("  logNoise "); Serial.print(logNoise);
//Serial.print(  "  vuLevel "); Serial.println(vuLevel);


#endif //USE_LOGS

#ifdef THRESHOLDING
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
#endif //THRESHOLDING
}
