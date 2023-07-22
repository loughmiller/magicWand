#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>

// I2C SCANNER
#define WIRE Wire

// COLOR SENSOR
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
uint8_t readHue();
uint8_t calcHue(float r, float g, float b);
byte stolenHue;

///////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////
void setup() {
  // SERIAL SETUP
  Serial.begin(9600);
  while(!Serial && millis() < 5000);
  delay(100);
  Serial.println("setup");

  WIRE.begin();

  // COLOR SENSOR SETUP
  if (tcs.begin()) {
    // turn off flash
    tcs.setInterrupt(true);
    Serial.println("Found color sensor");

  } //else {
  //   Serial.println("Color sensor not found");
  // }
}


// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  stolenHue = readHue();
  Serial.print("Hue: ");
  Serial.println(stolenHue);

  delay(5000);           // wait 5 seconds for next scan
}

///////////////////////////////////////////////////////////////////
// COLOR SENSOR FUNCTIONS
///////////////////////////////////////////////////////////////////

uint8_t readHue() {
  uint16_t clear, red, green, blue;
  float r, g, b;

  tcs.setInterrupt(false);  // LED ON
  delay(100);
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);   // LED OFF

  r = (float)red / (float)clear;
  g = (float)green / (float)clear;
  b = (float)blue / (float)clear;

  return calcHue(r, g, b);
}

uint8_t calcHue(float r, float g, float b) {
  float minC, maxC, delta, hue;

  minC = min(r, min(g, b));
  maxC = max(r, max(g, b));
  delta = maxC - minC;

  if(r == maxC) {
    hue = ( g - b ) / delta;
  } else if (g == maxC) {
    hue = 2 + (b - r) / delta;
  } else {
    hue = 4 + (r - g) / delta;
  }

  hue *= 60; // degrees
  if( hue < 0 ) {
    hue += 360;
  }

  return (uint8_t)((hue/360) * 255);
}

///////////////////////////////////////////////////////////////////
// \ COLOR SENSOR FUNCTIONS END /
///////////////////////////////////////////////////////////////////


