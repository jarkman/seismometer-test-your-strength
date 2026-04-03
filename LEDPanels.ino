

/******************************************************************************
This is an Arduino sketch to drive 4 LED panels based on MBI5034 LED drivers.

Written by Oliver Dewdney and Jon Russell

Adapted by jarkman, 3/26, for ESP32

Runs on an Feather ESP32-S3 2MB PSRAM
Select esp32/Feather ESP32-S3 2MB PSRAM

Panel details

https://led.limehouselabs.org/docs/tiles/led-tiles-v2/

https://wiki.london.hackspace.org.uk/view/LED_tiles_V2

Discussion of how to drive them
https://emalliab.wordpress.com/2025/11/01/hackspace-led-panels/

Panel connector is
D1  Lat  A1  NC
D2  OE   A0  Clk

PCB edge connector from top is

Gnd
Clk
LAT
OE
A1
A0
D2
D1

Basic Operation:

The objective was to be able to drive four panels with a single Arduino. 
Each panel has two data lines, so four panels require 8 data lines, i.e. a 
single byte.

An Arduino Micro was selected, as it has a native USB port and slightly more
RAM than an Uno. A Mega would probably work well too.

The code has a frame buffer in RAM with 4 sets of 384 bits 
(1 bank = 64 LEDs x 2 Rows x 3 bits (RGB) = 384) for each data line. 
Four panels, two data lines each, means all four panels can be driven by a byte 
wide frame buffer, assuming 3 bit colour. This means the update can be very 
efficient. The frame buffer is about 1.5KB so fits happily in to the ATMega32U4
with some room for local variables.

The UpdateFrame loop iterates over the line of 384 bits of serial data from the
frame buffer and clocks them out quickly.

Ideally, we needed a contiguous port on the Microcontroller to be attached to 
8 external data lines for the four panels. But most Arduinos don’t have this. 
On the Arduino Micro, PortB has the RX LED on bit 0 and PortD has the TX LED on
bit 5. So, I have connected 4 data lines to the high nibble on PortB and 4 data
lines to the low nibble on PortD.

If we had a contiguous port free (If we used a Mega) we could use only one port
and the UpdateFrame loop would be 1 instruction faster ! :-) But currently I 
copy the data to both ports, ignoring the high and low nibbles I don’t need.

UpdateFrame is called by an interrupt 390 times a second 
(OCR1A = 160; // compare match register 16MHz/256/390Hz)). 
UpdateFrame needs to be called 4 times to update the entire panel, as the panel
is split in to 4 rows of 128 LEDs (as 2 rows of 64).

For each half a panel (one data line) there are 8 rows of 64 LEDs, addresses in 
banks. Bank 0x00 is row 0&4,  Bank 0x01 is row 1&5, Bank 0x02 is row 2&6, Bank 
0x03 is row 3&7.

Each call updates one bank and leaves it lit until the next interrupt.

This method updates the entire frame (1024 RGB LEDs) about 100 times a second.

Port map for Arduino Micro (ATMega32U4)
    7     6     5     4     3     2     1     0
PB  D11   D10   D9    D8    MISO  MOSI  SCK   RX/SS
PC  D13   D5    X     X     X     X     X     X
PD  D6    D12   TX    D4    D1    D0    D2    D3
PE  X     D7    X     X     X     HWB   X     X
PF  A0    A1    A2    A3    X     X     A4    A5

This sketch now inherits from the Adafruit GFX library classes. This means we
can use the graphics functions like drawCircle, drawRect, drawTriangle, plus
the various text functions and fonts.
******************************************************************************/


#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include "FastPin.h"

// Running without interrupts, with everything done from loop(), ought to be smoother but 
// I get some odd uneven brightness effects I do not understand.
// The last bank to be updated, bank 3, is the only bright one. The other three are super-dim. Weird.
#define USE_INTERRUPTS
#define TIME_DRAWING

class LedPanel : public Adafruit_GFX
{
  public:
    LedPanel() : Adafruit_GFX(64,64) {};
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    uint16_t newColor(uint8_t red, uint8_t green, uint8_t blue);
    uint16_t getColor() { return textcolor; }
    void drawBitmapMem(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
};

LedPanel panel;

#define PIN_D1    5   //PD0 - D1 on Panel 1
#define PIN_D2    6   //PD1 - D2 on Panel 1
#define PIN_D3    9   //PD2 - D1 on Panel 2
#define PIN_D4    10   //PD3 - D2 on Panel 2

#define PIN_D5    11   //PB4 - D1 on Panel 3
#define PIN_D6    12   //PB5 - D2 on Panel 3
#define PIN_D7    13  //PB6 - D1 on Panel 4
#define PIN_D8    A5  //PB7 - D2 on Panel 4

#define PIN_A0    A0  //PF1 - A0 on all Panels
#define PIN_A1    A1  //PF6 - A1 on all Panels
#define PIN_CLK   A2  //PF4 - CLK on all Panels
#define PIN_LAT   A3  //PF5 - LAT on all Panels
#define PIN_OE    A4  //PF0 - OE on all Panels

FastPin pin_d1(PIN_D1, OUTPUT);
FastPin pin_d2(PIN_D2, OUTPUT);
FastPin pin_d3(PIN_D3, OUTPUT);
FastPin pin_d4(PIN_D4, OUTPUT);
FastPin pin_d5(PIN_D5, OUTPUT);
FastPin pin_d6(PIN_D6, OUTPUT);
FastPin pin_d7(PIN_D7, OUTPUT);
FastPin pin_d8(PIN_D8, OUTPUT);
FastPin pin_a0(PIN_A0, OUTPUT);
FastPin pin_a1(PIN_A1, OUTPUT);
FastPin pin_clk(PIN_CLK, OUTPUT);
FastPin pin_lat(PIN_LAT, OUTPUT);
FastPin pin_oe(PIN_OE, OUTPUT);

byte frame[4][384];

// esp32 timer bits from https://docs.espressif.com/projects/arduino-esp32/en/latest/api/timer.html
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
volatile uint32_t lastIsrDurationMicros = 0;

uint32_t drawDuration = 0;

volatile bool flagUpdatingFrame = false;
volatile bool flagDrawing = false;

// we can't use random for every pixel, so here's a collection of random numbers in the range 0-99 we can use quickly
#define NUM_RANDOMS 76
int randoms[NUM_RANDOMS];

int nextRandom = 0;

void initRandoms()
{
  for(int i = 0; i<NUM_RANDOMS;i++)
    randoms[i] = random(100);
}

int fastRandom()
{
  nextRandom = nextRandom%NUM_RANDOMS;
  return randoms[nextRandom++];
}

void FillBuffer(byte b){
  for(uint8_t x=0; x<4; x++){
    for(uint16_t y=0; y<384; y++){
      frame[x][y] = b;
    }
  }
}

void LedPanel::drawPixel(int16_t x, int16_t y, uint16_t color) {
  setpixel(x,y,color);
}

uint16_t LedPanel::newColor(uint8_t red, uint8_t green, uint8_t blue) {
  return (blue>>7) | ((green&0x80)>>6) | ((red&0x80)>>5);
}

void LedPanel::drawBitmapMem(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
  int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(bitmap[j * byteWidth + i / 8] & (128 >> (i & 7))) {
        panel.drawPixel(x+i, y+j, color);
      }
    }
  }
}

// bb describes which data lines drive which of the 4 panels.
// By adjusting the order of the bits in the array, you can change the panel order visually.
//byte bb[8] = { 0x40, 0x80, 0x10, 0x20, 0x04, 0x08, 0x01, 0x02 };
byte bb[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

// Set a pixel to a specific 3 bit colour (8 colours)
// 0b000 = black (off), 0b001 = Blue, 0b010 = Green, 0b100 = Red, 0b011 = Cyan, 0b101 = Magenta, 0b110 = Yellow, 0b111 = White, etc.
void setpixel(byte x, byte y, byte col) {
  if( x<0 || y<0 || x>=panel.width() || y>=panel.height())
    return;

  int16_t off = (x&7) + (x & 0xf8)*6 + ((y & 4)*2);
//  int16_t off = (x&7) + (x >> 3)*48 + ((y & 4)*2);
  byte row = y & 3;
  byte b = bb[(y&0x3f) >> 3];
  byte *p = & frame[row][off];
  for(byte c = 0; c < 3;c++) {
    if ( col & 1 ) {
      *p |= b;
    } else {
      *p &= ~b;
    }
    col >>= 1;
    p += 16;
  }
}

uint8_t bank = 0;

void UpdateFrame() {

  if( flagDrawing )
    return;



  long isrStart = micros();
  flagUpdatingFrame = true;

#ifdef TIME_DRAWING
  if( isrCounter % 500 == 0 )
  {
    Serial.print("updateFrame took ");
    Serial.print(lastIsrDurationMicros);
    Serial.print("us, draw took ");
    Serial.print(drawDuration);
    Serial.print("us, ");
    Serial.println(isrCounter);
  }
#endif

  byte * f = frame[bank];
  for (uint16_t n = 0; n<384; n++) {


    byte b1 = *f;
    if( false && isrCounter % 500 == 0 )
    {

      Serial.print(0!=(b1&0b0001));
      Serial.print(" ");
      Serial.print(0!=(b1&0b0010));
      Serial.print(", ");
    }

    pin_d1.write( 0!= (b1&0b0001));
    pin_d2.write( 0!= (b1&0b0010));
    pin_d3.write( 0!= (b1&0b0100));
    pin_d4.write( 0!= (b1&0b1000));
    pin_d5.write( 0!= (b1&0b00010000));
    pin_d6.write( 0!= (b1&0b00100000));
    pin_d7.write( 0!= (b1&0b01000000));
    pin_d8.write( 0!= (b1&0b10000000));

    f++;
    
    pin_clk.write(false);
    pin_clk.write(true);

    }


  pin_oe.write(true);
  pin_a0.write(bank & 0x01);
  pin_a1.write(bank & 0x02);

  pin_lat.write(true);
  pin_lat.write(false);

  pin_oe.write(false);
 
  
  long isrEnd = micros();

  lastIsrDurationMicros = isrEnd-isrStart;
  flagUpdatingFrame = false;
}


void setupLEDPanels() {
 
  initRandoms();

  //FillBuffer(0xFF);         // Set all LEDs on. (White)
  FillBuffer(0x00);         // Set all LEDs off. (Black)

#ifdef USE_INTERRUPTS
  // initialize Timer 
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 1Mhz
  int32_t timerHz = 1000000;
  timer = timerBegin(timerHz);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // frame update takes ~330us so we can update at 3kz if we want! No p0int doing it faster than 3x the draw update rate though
  int32_t hz = 400;
  int32_t interval = timerHz/hz;
  timerAlarm(timer, interval, true, 0);
#endif // USE_INTERRUPTS
}

#ifdef USE_INTERRUPTS
void ARDUINO_ISR_ATTR onTimer() {
  

  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter = isrCounter + 1;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output

  
  UpdateFrame();
  bank++;
  if(bank>3)
    bank = 0;
  
}
#endif //USE_INTERRUPTS

#define LED_BLACK 0
#define LED_BLUE 1
#define LED_GREEN 2
#define LED_CYAN 3
#define LED_RED 4
#define LED_MAGENTA 5
#define LED_YELLOW 6
#define LED_WHITE 7

void testText1to8() {
  FillBuffer(0x00);
  panel.setTextSize(1);
  panel.setCursor(0, 0);

  panel.setTextColor(LED_WHITE);  
  panel.println("1234567890");
  panel.setTextColor(LED_BLUE);  
  panel.println("ABCDEFGHIJ");
  panel.setTextColor(LED_GREEN);  
  panel.println("The Quick ");
  panel.setTextColor(LED_CYAN);  
  panel.println("Brown Fox ");
  panel.setTextColor(LED_RED);  
  panel.println("Jumped    ");
  panel.setTextColor(LED_MAGENTA);  
  panel.println("Over the  ");
  panel.setTextColor(LED_YELLOW);  
  panel.println("Lazy Dog !");
  panel.setTextColor(LED_WHITE);  
  panel.println("# * @ ! ; ");
}

/*
void testText() {
  panel.fillScreen(LED_BLACK);
  panel.setCursor(0, 0);
  panel.setTextColor(LED_WHITE);  
  panel.setTextSize(1);
  panel.println("Hello!");
  panel.setTextColor(LED_YELLOW); 
  panel.setTextSize(1);
  panel.println(1234.56);
  panel.setTextColor(LED_RED);    
  panel.setTextSize(1);
  panel.println(0xDEADBEEF, HEX);
  panel.setTextColor(LED_GREEN);
  panel.setTextSize(1);
  panel.println("Groop");
  panel.println("I implore thee...");
}

void testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = panel.width(), h = panel.height();

  panel.fillScreen(LED_BLACK);
  for(y=0; y<h; y+=4) panel.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=4) panel.drawFastVLine(x, 0, h, color2);
}
}
*/
void testFilledRects(uint16_t color1, uint16_t color2) {
  int n, i, i2,
    cx = panel.width()  / 2 - 1,
    cy = panel.height() / 2 - 1;

  panel.fillScreen(LED_BLACK);
  n = min(panel.width(), panel.height());
  for(i=n; i>0; i-=6) {
    i2 = i / 2;
    panel.fillRect(cx-i2, cy-i2, i, i, color1);
    panel.drawRect(cx-i2, cy-i2, i, i, color2);
  }
}

void testFilledCircles(uint8_t radius, uint16_t color) {
  int x, y, 
    w = panel.width(), 
    h = panel.height(), 
    r2 = radius * 2;

  panel.fillScreen(LED_BLACK);
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      panel.fillCircle(x, y, radius, color);
    }
  }
}

void testTriangles() {
  int n, i, 
    cx = panel.width()  / 2 - 1,
    cy = panel.height() / 2 - 1;

  panel.fillScreen(LED_BLACK);
  n = min(cx, cy);
  for(i=0; i<n; i+=4) {
    panel.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      LED_RED);
  }
}

void testRoundRects() {
  int w, i, i2,
    cx = panel.width()  / 2 - 1,
    cy = panel.height() / 2 - 1;

  panel.fillScreen(LED_BLACK);
  w = min(panel.width(), panel.height());
  for(i=0; i<w; i+=4) {
    i2 = i / 2;
    panel.drawRoundRect(cx-i2, cy-i2, i, i, i/8, LED_YELLOW);
  }
}


/*
//#include <fonts/FreeSerifBoldItalic9pt7b.h>
void testFonts(){
  panel.fillScreen(LED_BLACK);
  panel.setFont(&FreeSerifBoldItalic9pt7b);
  panel.setCursor(2, 16);
  panel.setTextColor(LED_GREEN);  
  panel.setTextSize(1);
  panel.println("Hello");

  panel.setFont();
  panel.setTextColor(LED_WHITE);  
  panel.setCursor(2, 48);
  panel.println("FreeSerifBoldItalic9pt7b");
  delay(10000);
}
*/

/*
const unsigned char LHSlogoBitmap [] PROGMEM = {
  // 'Hackspace64x64'
  0xff, 0xff, 0xff, 0xc0, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x3f, 0xff, 0xff, 
  0xff, 0xff, 0xe0, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x01, 0xff, 0xff, 
  0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 
  0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 
  0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0xff, 
  0xff, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x7f, 
  0xfe, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x3f, 
  0xf8, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x1f, 
  0xf0, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x07, 
  0xe0, 0x00, 0x03, 0xff, 0xfe, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xff, 0xff, 0x00, 0x00, 0x03, 
  0xc0, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x03, 0xc0, 0x00, 0x11, 0xff, 0xff, 0xc0, 0x00, 0x03, 
  0x80, 0x00, 0x38, 0xff, 0xff, 0xe0, 0x00, 0x01, 0x80, 0x00, 0x7c, 0x7f, 0xff, 0xf0, 0x00, 0x01, 
  0x80, 0x00, 0xfe, 0x3f, 0xff, 0xf8, 0x00, 0x01, 0x80, 0x01, 0xff, 0x1c, 0xff, 0xfd, 0x80, 0x01, 
  0x00, 0x03, 0xff, 0x88, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x07, 0xff, 0xc1, 0xff, 0xff, 0xe0, 0x00, 
  0x00, 0x0f, 0xff, 0xe3, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x1f, 0xff, 0xc7, 0xff, 0xff, 0xf8, 0x00, 
  0x00, 0x3f, 0xff, 0x8f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x7f, 0xff, 0x9f, 0xff, 0xff, 0xfe, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xfd, 0xff, 0xfe, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xf9, 0xff, 0xfc, 0x00, 
  0x00, 0x1f, 0xff, 0xff, 0xf3, 0xff, 0xf8, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xe7, 0xff, 0xf0, 0x00, 
  0x00, 0x07, 0xff, 0xff, 0xc3, 0xff, 0xe0, 0x00, 0x00, 0x03, 0xff, 0xff, 0x91, 0xff, 0xc0, 0x00, 
  0x80, 0x01, 0xbf, 0xff, 0x38, 0xff, 0x80, 0x01, 0x80, 0x00, 0x1f, 0xff, 0xfc, 0x7f, 0x00, 0x01, 
  0x80, 0x00, 0x0f, 0xff, 0xfe, 0x3e, 0x00, 0x01, 0x80, 0x00, 0x07, 0xff, 0xff, 0x1c, 0x00, 0x01, 
  0xc0, 0x00, 0x03, 0xff, 0xff, 0x88, 0x00, 0x03, 0xc0, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x03, 
  0xc0, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x00, 0x07, 
  0xe0, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x0f, 
  0xf0, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x1f, 
  0xfc, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x3f, 0xfe, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x7f, 
  0xfe, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0xff, 
  0xff, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 
  0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 
  0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 
  0xff, 0xff, 0x80, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x07, 0xff, 0xff, 
  0xff, 0xff, 0xfc, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x03, 0xff, 0xff, 0xff, 
};
*/

// draw a blob with a solid center and a probabilistic dither to the outside
void fuzzyBlob(int16_t x0, int16_t y0, int16_t dx, int16_t dy, uint16_t color)
{
  int w = panel.width();
  int h = panel.height();

  for(int16_t x = x0-dx; x <= x0+dx; x++)
  {
    for( int16_t y = y0-dy; y<= y0+dy; y++)
    {

      int32_t probability = 10000 - (((100 * abs(x-x0))/(dx)) * ((100 * abs(y-y0))/(dy)));
      /*
      Serial.print(x-x0);
      Serial.print(", ");
      Serial.print(y-y0);
      Serial.print(" - ");
      
      Serial.println(probability);
      */
      
      if( fastRandom() <probability/100)
        panel.drawPixel(x%w, y%h, color);
    }
  }
  
}

// draw a bar all the way across the x direction at the specified 
void fuzzyBar(int16_t x0, int16_t dx, uint16_t color)
{
  int w = panel.width();
  int h = panel.height();

 
  for(int16_t x = x0-dx; x < x0+dx; x++)
  {
    for( int16_t y = 0; y<=h; y++)
    {
      float probability = 100 -  (100*abs(x-x0)/(dx));
      
      if( fastRandom()<probability)
        panel.drawPixel(x%w, y%h, color);
    }
  }
}
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void bouncer()
{

  float pos;

  //float t = millis()/5000.0;
  float s = vuLevel; //sin(t*2.0*3.1415);

  pos = fmap(s, 0, 1.0, 0, panel.width()-10);

  float fatness = fmap(pos,5,panel.width()-10,2,15);

  //Serial.print("s "); Serial.print(s); Serial.print(" pos "); Serial.println(pos);
  fuzzyBar(pos, fatness, LED_MAGENTA);
  fuzzyBar(pos+fatness/2, fatness/2, LED_CYAN);

}
// a moving light near the top to stop people walking into it
void spinner()
{
  static float pos = 0;
  static float rps = 0.5; // revs per sec
  int w = panel.width();
  int h = panel.height();
  float pps = rps*w;  // pixel per sec

  static int32_t lastT = 0;
  int32_t now = micros();

  pos = pos + (now-lastT)*0.000001*pps;
  pos = fmod(pos,w);
  lastT = now;

  fuzzyBlob(w-5, (int)pos, 4,8, LED_RED);
  /*
  // short vertical line
  for( int x =w-1; x>w-10; x--)
  {
    
    panel.drawPixel(x, (int)pos,LED_RED);
    panel.drawPixel(x, ((int)pos+1)%h,LED_RED);
  }
  */
  
}

void draw() // currently takes around 1ms
{
  if( flagUpdatingFrame ) // don't draw during update, to avoid tearing
    return;

  long startT = micros();

  flagDrawing = true;
  

  FillBuffer(0x00);

  bouncer();

  spinner();
  
  drawDuration = micros()-startT;

  flagDrawing = false;

}

long lastDraw = 0;
long lastUpdate = 0;

long drawInterval = 20000;
long updateInterval = 25000;

void loopLEDPanels(){

  long now = micros();

  if( now-lastDraw>drawInterval)
  {
    lastDraw = now;
    draw();


  }

#ifndef USE_INTERRUPTS
if(now-lastUpdate>updateInterval)
{
  lastUpdate = now;
  //for( bank = 0; bank < 4; bank ++) // do all 4 banks
    UpdateFrame(); 
    bank++;
    if(bank>3) bank = 0;
}
#endif

 
  // assorted useful tests
  //testText1to8();
  //panel.drawCircle(5,5,5,LED_WHITE);
  //testText();
  //testFastLines(LED_RED, LED_BLUE);
  //testTriangles();
  //testRoundRects();
  //testFilledRects(LED_CYAN, LED_MAGENTA);
  //testFilledCircles(8, LED_GREEN);
  //testFonts();
  //panel.drawBitmap(0, 0, (const uint8_t *)&LHSlogoBitmap, 64, 64, LED_WHITE, LED_BLUE);

  //FillBuffer(0xFF); 		// turn all LED's on


}



