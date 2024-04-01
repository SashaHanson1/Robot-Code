// 
// MSE 2202 TCS34725 colour sensor example
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Michael Naish
//  Date:     2024 03 05 
//

//#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();
long degreesToDutyCycle(int deg);

// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725   

const int CollectorServoPin       = 39;
const int ArmServoPin             =40;
const int AccepterServoPin          = 41;                 // GPIO pin for servo motor
const int GumballServoPin         = 42;                 // GPIO pin for servo motor
const int CollectorServoChannel       = 4;
const int ArmServoChannel         = 7;
const int AccepterServoChannel      = 5;                  // PWM channel used for the RC servo motor 
const int GumballServoChannel      = 6;                  // PWM channel used for the RC servo motor 

// Variables
boolean heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds
int servoPos;                                      // desired servo angle
unsigned long previousMillis = 0;  // Store the last time the action was executed
unsigned long newMillis = 0;
unsigned long collectorMillis = 0; 
unsigned long armMillis = 0;
unsigned long swingingMillis = 0;
bool change = false;
bool arm = false;// tells us if the collector arm is closed or open
bool onGround = true;

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
  //set up for servo
  pinMode(AccepterServoPin, OUTPUT);                      // configure servo GPIO for output
  pinMode(GumballServoPin, OUTPUT);
  pinMode(ArmServoPin, OUTPUT);
  pinMode(CollectorServoPin, OUTPUT);
  ledcSetup(AccepterServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcSetup(GumballServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcSetup(ArmServoChannel, 50, 14);
  ledcSetup(CollectorServoChannel, 50, 14);
  ledcAttachPin(AccepterServoPin, AccepterServoChannel);         // assign servo pin to servo channel
  ledcAttachPin(GumballServoPin, GumballServoChannel);
  ledcAttachPin(CollectorServoPin, CollectorServoChannel);
  ledcAttachPin(ArmServoPin, ArmServoChannel);

   //set up of push button
  pinMode(0, INPUT_PULLUP);

}

void loop() {
  uint16_t r, g, b, c;                                // RGBC values from TCS34725

  unsigned long currentMillis = millis();  // Get the current time
  // Check if it's time to perform the action
  if (currentMillis - previousMillis >= 3000) {
    // Save the last time the action was executed
    previousMillis = currentMillis;
    change = true;
    ledcWrite(AccepterServoChannel, degreesToDutyCycle(90));
  } else if (change && currentMillis - previousMillis >= 100){
    ledcWrite(AccepterServoChannel, degreesToDutyCycle(180));
    change = false;
  }
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
    if (g > r && g > b && g > 60 && g < 90 && c < 200 ){//this indiactes that the color sensor has found a green gem
      ledcWrite(GumballServoChannel, degreesToDutyCycle(0)); // turn to sorted side
      newMillis = currentMillis;
      Serial.printf("green detected");

    } 
    else if(currentMillis - newMillis >= 250){
      ledcWrite(GumballServoChannel, degreesToDutyCycle(180));
    }
  } 
#ifdef PRINT_COLOUR            
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif

  if((currentMillis - collectorMillis) >= 15000 || !onGround){
    ledcWrite(CollectorServoChannel, degreesToDutyCycle(180));//closed position
    onGround = false;
    ledcWrite(ArmServoChannel, degreesToDutyCycle(120));// top position
    if ((currentMillis - collectorMillis) >= 15000) {
      armMillis = currentMillis; 
    }
    if (currentMillis - armMillis >= 3000){
      ledcWrite(ArmServoChannel, degreesToDutyCycle(0));//bottom position
      onGround = true;
    }
    collectorMillis = currentMillis; 
  } else if (onGround && (currentMillis - swingingMillis) >= 3000) {
    ledcWrite(CollectorServoChannel, degreesToDutyCycle(180));//closed position
    swingingMillis = currentMillis;
    Serial.printf("Swinging time: %d\n", swingingMillis);
  } else if (onGround && currentMillis - swingingMillis >= 250) {
    ledcWrite(CollectorServoChannel, degreesToDutyCycle(90));// open position
  }
  
  doHeartbeat();                                      // update heartbeat LED
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}

long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                     // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                    // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;              // (dutyCycle / 16383) * 100
#endif

  return dutyCycle;
}
