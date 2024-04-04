/*
########################## INTEGRATE PID SYSTEM FROM LAB 2 #####################################
 */

//  To program and use ESP32-S3
//   
//  Tools->:
//  Board: "Adafruit Feather ESP32-S3 No PSRAM"
//  Upload Speed: "921600"
//  USB CDC On Boot: "Enabled"
//  USB Firmware MSC on Boot: "Disabled"
//  USB DFU On Bot: "Disabled"
//  Upload Mode:"UART0/Hardware CDC"
//  SPU Frequency: "240MHz (WiFi)"
//  Flash Mode: "QIO 80MHz"
//  Flash SIze: "4MB (32Mb)"
//  Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//  Core Debug Level: "Verbose"
//  PSRAM: 'Disabled"
//  Arduino Runs On: "Core 1"
//  Events Run On: "Core 1"
//
//  To program, press and hold the reset button then press and hold program button, release the reset button then 
//  release the program button 
//

/* Task:

*/

// Uncomment keywords to enable debugging output
//#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void Indicator();  // for mode/heartbeat on Smart LED
void setTarget(int dir, double distance);
void servoReset();
void doHeartbeat();
long degreesToDutyCycle(int deg);

// Port pin constants
#define LEFT_MOTOR_A 35        // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 36        // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 37       // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 38       // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A 15      // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B 16      // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A 11     // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B 12     // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON 0          // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1 1               // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED 21           // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT 1      // number of SMART LEDs in use
#define IR_DETECTOR 14         // GPIO14 pin 17 (J14) IR detector input
#define PRINT_COLOUR

// Constants
const int cDisplayUpdate = 100;           // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                    // bit resolution for PWM
const int cMinPWM = 150;                  // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;  // PWM value for maximum speed

const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725   

const int DoorServoPin          = 41;     // Do not think this pin works for servos
const int GumballServoPin         = 42;                 // GPIO pin for servo motor
const int DoorServoChannel      = 6;                  // PWM channel used for the RC servo motor 
const int GumballServoChannel      = 7;                  // PWM channel used for the RC servo motor 
const long ArmTiming = 15000;

const int cRightAdjust = 0;
const int cLeftAdjust = 2;
const long cCountsRev = 1096;
const double distancePerRev = 13.8;  //circumference of the wheels in cm

const float pi = 3.14159;                                                        // pi is used to calculate circumference of a wheel based on its diameter
const int cDist = cCountsRev / (4.2 * pi);                                      // this is the calculated unit length to travel
int pauseTime = 0;                                                             // used to determine when to pause the robots motion for a moment after return to its start position

//=====================================================================================================================

// Variables
boolean motorsEnabled = true;         // motors enabled flag
boolean timeUp3sec = false;           // 3 second timer elapsed flag
boolean timeUp2sec = false;           // 2 second timer elapsed flag
boolean timeUp200msec = false;        // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;         // motor drive speed (0-255)
unsigned char rightDriveSpeed;        // motor drive speed (0-255)
unsigned char driveIndex;             // state index for run mode
unsigned int modePBDebounce;          // pushbutton debounce timer count
unsigned long timerCount3sec = 0;     // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;     // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;  // 200 millisecond timer count in milliseconds
unsigned long displayTime;            // heartbeat LED update timer
unsigned long previousMicros;         // last microsecond count
unsigned long currentMicros;          // current microsecond count

unsigned long prevTime = 0;
double target = 0;          // target pos for encoder
bool servoReseted = false;  // flag for servo reset
bool calibrated = false;
int armServoPos = 1000;
char IRData = ' ';

unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds
int servoPos;                                      // desired servo angle
unsigned long currentMillis = 0;    // Stores the current time
unsigned long previousMillis = 0;  // Store the last time the action was executed
unsigned long newMillis = 0;
unsigned long delayMillis = 0;
unsigned long collectorMillis = 0; 
unsigned long armMillis = 0;
unsigned long swingingMillis = 0;
unsigned long resetMillis = 0;
bool change = false;
bool arm = false;// tells us if the collector arm is closed or open
bool onGround = true;
int gumballIndex = 0;
int baseState = 45;
bool green = false;

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255,
                                        240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15 };

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

unsigned int robotModeIndex = 0;  // robot operational state
unsigned int modeIndicator[6] = {
  // colours for different modes
  SmartLEDs.Color(255,0,0),                                      //   red - stop
   SmartLEDs.Color(0,255,0),                                     //   green - run
   SmartLEDs.Color(0,0,255),                                     //   blue - empty case
   SmartLEDs.Color(255,255,0),                                   //   yellow - empty case
   SmartLEDs.Color(0,255,255),                                   //   cyan - empty case
   SmartLEDs.Color(255,0,255)                                    //   magenta - empty case
};

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();               // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();   // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();  // Instance of Encoders for right encoder data
IR Scan = IR();                      // instance of IR for detecting IR signals

void setup() {
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200);
#endif

  // Set up motors and encoders
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B);  // set up motors as Drive 1
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning);       // set up left encoder
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning);   // set up right encoder

  Scan.Begin(IR_DETECTOR, 1200);  //set up IR Detection @ 1200 baud

  // Set up SmartLED
  SmartLEDs.begin();                                     // initialize smart LEDs object (REQUIRED)
  SmartLEDs.clear();                                     // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));  // set pixel colors to 'off'
  SmartLEDs.show();                                      // send the updated pixel colors to the hardware

  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);  // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);          // Set up mode pushbutton
  modePBDebounce = 0;                          // reset debounce timer count

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
  pinMode(DoorServoPin, OUTPUT);                      // configure servo GPIO for output
  pinMode(GumballServoPin, OUTPUT);
  ledcSetup(DoorServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcSetup(GumballServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  
  ledcAttachPin(DoorServoPin, DoorServoChannel);         // assign servo pin to servo channel
  ledcAttachPin(GumballServoPin, GumballServoChannel);

   //set up of push button
  pinMode(0, INPUT_PULLUP);
}

void loop() {
  long pos[] = { 0, 0 };  // current motor positions
  int pot = 0;            // raw ADC value from pot
  uint16_t r, g, b, c;    // RGBC values from TCS34725

  currentMillis = millis();  // Get the current time
  currentMicros = micros();                        // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
    previousMicros = currentMicros;                // record current time in microseconds

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec = timerCount3sec + 1;  // increment 3 second timer count
    if (timerCount3sec > 3000) {          // if 3 seconds have elapsed
      timerCount3sec = 0;                 // reset 3 second timer count
      timeUp3sec = true;                  // indicate that 3 seconds have elapsed
    }

    // 2 second timer, counts 2000 milliseconds
    timerCount2sec = timerCount2sec + 1;  // increment 2 second timer count
    if (timerCount2sec > 2000) {          // if 2 seconds have elapsed
      timerCount2sec = 0;                 // reset 2 second timer count
      timeUp2sec = true;                  // indicate that 2 seconds have elapsed
    }

    // 200 millisecond timer, counts 200 milliseconds
    timerCount200msec = timerCount200msec + 1;  // Increment 200 millisecond timer count
    if (timerCount200msec > 200)                // If 200 milliseconds have elapsed
    {
      timerCount200msec = 0;  // Reset 200 millisecond timer count
      timeUp200msec = true;   // Indicate that 200 milliseconds have elapsed
    }

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON)) {  // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (modePBDebounce <= 25) {             // 25 millisecond debounce time
        modePBDebounce = modePBDebounce + 1;  // increment debounce timer count
        if (modePBDebounce > 25) {            // if held for at least 25 mS
          modePBDebounce = 1000;              // change debounce timer count to 1 second
        }
      }
      if (modePBDebounce >= 1000) {  // maintain 1 second timer count until release
        modePBDebounce = 1000;
      }
    } else {                       // pushbutton GPIO goes HIGH (nominal release)
      if (modePBDebounce <= 26) {  // if release occurs within debounce interval
        modePBDebounce = 0;        // reset debounce timer count
      } else {
        modePBDebounce = modePBDebounce + 1;    // increment debounce timer count
        if (modePBDebounce >= 1025) {           // if pushbutton was released for 25 mS
          modePBDebounce = 0;                   // reset debounce timer count
          robotModeIndex++;                     // switch to next mode
          robotModeIndex = robotModeIndex & 1;  // keep mode index between 0 and 7
          timerCount3sec = 0;                   // reset 3 second timer count
          timeUp3sec = false;                   // reset 3 second timer
        }
      }
    }

    // check if drive motors should be powered
    motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);  // if SW1-1 is on (low signal), then motors are enabled

    // modes
    // 0 = Default after power up/reset. Robot is stopped.
    // 1 = Press mode button once to enter.        Run robot
    switch (robotModeIndex) {
      case 0:  // Robot stopped
        Bot.Stop("D1");
        LeftEncoder.clearEncoder();  // clear encoder counts
        RightEncoder.clearEncoder();
        driveIndex = 0;        // reset drive index
        timeUp2sec = false;    // reset 2 second timer
        servoReseted = false;  //reset flag
        calibrated = false;
        break;

      case 1:              // Run robot
        // Read pot to update drive motor speed
        pot = analogRead(POT_R1);
        leftDriveSpeed = cMaxPWM - cLeftAdjust;
        rightDriveSpeed = cMaxPWM - cRightAdjust;
#ifdef DEBUG_DRIVE_SPEED
        Serial.print(F(" Left Drive Speed: Pot R1 = "));
        Serial.print(pot);
        Serial.print(F(", mapped = "));
        Serial.println(leftDriveSpeed);
#endif
#ifdef DEBUG_ENCODER_COUNT
        if (timeUp200msec) {
          timeUp200msec = false;              // reset 200 ms timer
          LeftEncoder.getEncoderRawCount();   // read left encoder count
          RightEncoder.getEncoderRawCount();  // read right encoder count
        }
#endif        
        digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
        if (tcsFlag) {                                      // if colour sensor initialized
          tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
          if (currentMillis - resetMillis >= 1000) { // only run  after slight delay
            switch (gumballIndex) {
              case 0:
                ledcWrite(GumballServoChannel, degreesToDutyCycle(baseState)); // Gumball base state
                if (currentMillis - newMillis >= 2000) {
                  if (g > r && g > b && g > 60 && g < 90 && c < 190 && c > 135) { //this indiactes that the color sensor has found a green gem
                    Serial.printf("green detected________________________________________________");
                    green = true;
                  } else {
                    green = false;
                  }
                  gumballIndex = 1;
                  delayMillis = currentMillis;
                }
                break;

              case 1:
                ledcWrite(GumballServoChannel, degreesToDutyCycle(77)); // acceptor state
                if (green) {
                  gumballIndex = 2;
                } else {
                  gumballIndex = 3;
                }
                break;
              
              case 2:
                ledcWrite(GumballServoChannel, degreesToDutyCycle(0)); // Gumball green bin state
                newMillis = currentMillis;
                gumballIndex = 0;
                baseState = 45;
                break;

              case 3:
                ledcWrite(GumballServoChannel, degreesToDutyCycle(170)); // Gumball garbage bin state
                newMillis = currentMillis;
                gumballIndex = 0;
                baseState = 120;
                break;
            }
            resetMillis = currentMillis;
          }
        } 

        if (currentMillis >= 100000) { // open door when path is complete (i.e. at base station)
          ledcWrite(DoorServoChannel, degreesToDutyCycle(90)); // Open Door and drop gems into colletion container
        } else {
          ledcWrite(DoorServoChannel, degreesToDutyCycle(0));
        }

#ifdef PRINT_COLOUR            
            Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif
        break;
    }

    // Update brightness of heartbeat display on SmartLED
    displayTime++;                                             // count milliseconds
    if (displayTime > cDisplayUpdate) {                        // when display update period has passed
      displayTime = 0;                                         // reset display counter
      LEDBrightnessIndex++;                                    // shift to next brightness level
      if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {  // if all defined levels have been used
        LEDBrightnessIndex = 0;                                // reset to starting brightness
      }
      SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
      Indicator();                                                       // update LED
    }
  }
}

// set target based on input for direction, and distance using the circumference of the wheel and rotations per revolution
void setTarget(int motor, int dir, double distance) {
  if (motor == 0) {
    LeftEncoder.getEncoderRawCount();
    if (dir == 1) {
      target = LeftEncoder.lRawEncoderCount + (distance / distancePerRev * cCountsRev);
    }
    if (dir == -1) {
      target = LeftEncoder.lRawEncoderCount - (distance / distancePerRev * cCountsRev);
    }
  } else {
    RightEncoder.getEncoderRawCount();
    if (dir == -1) {
      target = RightEncoder.lRawEncoderCount + (distance / distancePerRev * cCountsRev);
    }
    if (dir == 1) {
      target = RightEncoder.lRawEncoderCount - (distance / distancePerRev * cCountsRev);
    }
  }
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);  // set pixel colors to = mode
  SmartLEDs.show();                                           // send the updated pixel colors to the hardware
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