#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <WIFI.h>
#include <painlessMesh.h>
#include "AiEsp32RotaryEncoder.h"


////////////////////////////////////////////////////////////////////////////////
// WIFI
////////////////////////////////////////////////////////////////////////////////
#define   MESH_PREFIX     "wizardMesh"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

uint32_t newConnection = 0;
uint32_t lightning = 0;
void newConnectionCallback(uint32_t nodeId);
void lightningAnimation();
bool connected = false;


////////////////////////////////////////////////////////////////////////////////
// COLOR SENSOR
////////////////////////////////////////////////////////////////////////////////
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
uint8_t readHue();
uint8_t calcHue(float r, float g, float b);
byte stolenHue;
#define COLOR_SENSOR_POS 7
#define COLOR_SENSOR_NEG 34



////////////////////////////////////////////////////////////////////////////////
// ROTARY ENCODER
////////////////////////////////////////////////////////////////////////////////
#define ROTARY_ENCODER_A_PIN 4
#define ROTARY_ENCODER_B_PIN 5
#define ROTARY_ENCODER_BUTTON_PIN 3
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

void IRAM_ATTR readEncoderISR() {
	rotaryEncoder.readEncoder_ISR();
}

///////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////
void setup() {
  // SERIAL SETUP
  Serial.begin(9600);
  while(!Serial && millis() < 5000);
  delay(100);
  Serial.println("setup");

  // WIFI SETUP
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );

  // COLOR SENSOR SETUP
  pinMode(COLOR_SENSOR_POS, OUTPUT);
  pinMode(COLOR_SENSOR_NEG, OUTPUT);
  digitalWrite(COLOR_SENSOR_POS, HIGH);
  digitalWrite(COLOR_SENSOR_NEG, LOW);
  delay(100);

  if (tcs.begin()) {
    // turn off flash
    tcs.setInterrupt(true);
    Serial.println("Found color sensor");
  } else {
    Serial.println("Color sensor not found");
  }

  // ROTARY ENCODER SETUP
  Serial.println("init rotary encoder");
  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  // rotaryEncoder.readEncoder_ISR();
  bool circleValues = true;
  rotaryEncoder.setBoundaries(0, 255, circleValues);
  rotaryEncoder.setAcceleration(25);

  Serial.println("setup done");
  Serial.println("setup done");
  Serial.println("setup done");
}

///////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////
void loop() {

  mesh.update();

  // Serial.println(rotaryEncoder.readEncoder());

  if (rotaryEncoder.encoderChanged()) {
    uint_fast8_t location = rotaryEncoder.readEncoder();
    Serial.print("Value: ");
    Serial.println(location);

    stolenHue = readHue();
    Serial.print("Hue: ");
    Serial.println(stolenHue);
  }
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


