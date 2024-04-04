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

const int CollectorServoPin       = 40;
const int ArmServo1_Pin             = 41;
const int ArmServo2_Pin             = 42;
const int CollectorServoChannel       = 5;
const int ArmServo1_Channel         = 6;
const int ArmServo2_Channel         = 7;

int cRightAdjust = 0;
int cLeftAdjust = 30;

const long cCountsRev = 1096;
const double distancePerRev = 13.8;  //circumference of the wheels in cm

const int cIN1Pin = 35;                   // GPIO pin(s) for IN1
const int cIN1Chan = 0;                   // PWM channel for IN1
const int c2IN2Pin = 36;                  // GPIO pin for IN2
const int cIN2Chan = 1;                   // PWM channel for IN2
const int cMaxSpeedInCounts = 1600;       // maximum encoder counts/sec
const float kp = 1.5;                     // proportional gain for PID
const float ki = 2;                       // integral gain for PID
const float kd = 0.2;                     // derivative gain for PID
const float pi = 3.14159;                                                        // pi is used to calculate circumference of a wheel based on its diameter
const int cDist = cCountsRev / (4.2 * pi);                                      // this is the calculated unit length to travel
int pauseTime = 0;                                                             // used to determine when to pause the robots motion for a moment after return to its start position

// set servo limits
const float OpenAngle = 167;
const float ClosedAngle = 66;
const float LiftedAngle1 = 145;
const float DroppedAngle1 = 15;
const float LiftedAngle2 = 0;
const float DroppedAngle2 = 130;

float CollectorAngle = OpenAngle; // set the initial open position
float Arm1_Angle = DroppedAngle1; // set the initial ground position
float Arm2_Angle = DroppedAngle2; // set the initial ground position
bool closing = false;
bool opening = false;
bool lifting = false;
bool dropping = false;
bool routeComplete = false;
int servoCase = 0;
float length = 250;
int count = 0;
boolean reverse = false;

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
unsigned long collectorMillis = 0; 
unsigned long armMillis = 0;
unsigned long swingingMillis = 0;
unsigned long delayMillis = 0;
unsigned long lastTime = 0;                // last time of motor control was updated

bool change = false;
bool arm = false;// tells us if the collector arm is closed or open
bool onGround = true;
int turns = 0;

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

  //set up for servo
  pinMode(ArmServo1_Pin, OUTPUT);
  pinMode(ArmServo2_Pin, OUTPUT);
  pinMode(CollectorServoPin, OUTPUT);

  ledcSetup(ArmServo1_Channel, 50, 14);
  ledcSetup(ArmServo2_Channel, 50, 14);
  ledcSetup(CollectorServoChannel, 50, 14);

  ledcAttachPin(CollectorServoPin, CollectorServoChannel);
  ledcAttachPin(ArmServo1_Pin, ArmServo1_Channel);
  ledcAttachPin(ArmServo2_Pin, ArmServo2_Channel);

   //set up of push button
  pinMode(0, INPUT_PULLUP);
}

void loop() {
  float deltaT = 0;     // time interval
  long pos = 0;         // current motor positions
  long e = 0;           // position error
  float ePrev = 0;      // previous position error
  float dedt = 0;       // rate of change of position error (de/dt)
  float eIntegral = 0;  // integral of error
  float u = 0;          // PID control signal
  int pwm = 0;          // motor speed(s), represented in bit resolution
  int dir = 1;          // direction that motor should turn
  // store encoder position to avoid conflicts with ISR updates
  noInterrupts();     // disable interrupts temporarily while reading
  pos = ((LeftEncoder.lRawEncoderCount) + (RightEncoder.lRawEncoderCount)) / 2;  // read and store current motor position
  interrupts();       // turn interrupts back on

  target = length * cDist;

  unsigned long curTime = micros();                  // capture current time in microseconds
  if (curTime - lastTime > 10000) {                  // wait ~10 ms
    deltaT = ((float)(curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                              // update start time for next control cycle
    // use PID to calculate control signal to motor
    e = target - pos;                         // position error
    dedt = ((float)e - ePrev) / deltaT;       // derivative of error
    eIntegral = eIntegral + e * deltaT;       // integral of error (finite difference)
    u = kp * e + kd * dedt + ki * eIntegral;  // compute PID-based control signal
    ePrev = e;                                // store error for next control cycle

    // set direction based on computed control signal
    dir = 1;      // default to forward directon
    if (u < 0) {  // if control signal is negative
      dir = -1;   // set direction to reverse
    }

    // set speed based on computed control signal
    u = fabs(u);                  // get magnitude of control signal
    if (u > cMaxSpeedInCounts) {  // if control signal will saturate motor
      u = cMaxSpeedInCounts;      // impose upper limit
    }

    pwm = map(u, 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM);  // convert control signal to pwm    

    #ifdef SERIAL_STUDIO

    #endif
  }
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
    // 2 = Press mode button twice to enter.       // empty
    // 3 = Press mode button three times to enter. // empty
    // 4 = Press mode button four times to enter.  // empty
    // 5 = Press mode button five times to enter.  // empty
    // 6 = Press mode button six times to enter.   // empty
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
        leftDriveSpeed = cMaxPWM - cLeftAdjust;
        rightDriveSpeed = cMaxPWM - cRightAdjust;
#ifdef DEBUG_DRIVE_SPEED

#endif
#ifdef DEBUG_ENCODER_COUNT
        if (timeUp200msec) {
          timeUp200msec = false;              // reset 200 ms timer
          LeftEncoder.getEncoderRawCount();   // read left encoder count
          RightEncoder.getEncoderRawCount();  // read right encoder count
        }
#endif
        if (motorsEnabled) {          // run motors only if enabled
          switch (driveIndex) {     // cycle through drive states
            case 0: // stop robot
              Bot.Stop("D1");
              if (onGround) {
                driveIndex = 1;
              }
              break;

            case 1: // drive forward
              Bot.Forward("D1", rightDriveSpeed, leftDriveSpeed);
              if (abs(LeftEncoder.lRawEncoderCount) >= cDist * length) {
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 2;
                reverse = true;
              }
              break;

            case 2: // reverse into bin
              Bot.Reverse("D1", rightDriveSpeed, leftDriveSpeed);
              cRightAdjust = 5;
              cLeftAdjust = 0;
              if (abs(LeftEncoder.lRawEncoderCount) <= cDist * length) {
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                motorsEnabled = false; // end drive system as the objective is now complete
              }
          }
        } else {  // stop when motors are disabled
          Bot.Stop("D1");
        }

        if (currentMillis - delayMillis >= 10) { // only run  after slight delay
          switch(servoCase) {
            case 0: // collector arm base position
              ledcWrite(CollectorServoChannel, degreesToDutyCycle(CollectorAngle)); // open position
              ledcWrite(ArmServo1_Channel, degreesToDutyCycle(DroppedAngle1));// right servo top position
              ledcWrite(ArmServo2_Channel, degreesToDutyCycle(DroppedAngle2));// left servo top position
              if (onGround && currentMillis - swingingMillis >= 3000) {
                servoCase = 1;
              } else if (currentMillis - collectorMillis >= 9000) {
                onGround = false;
                servoCase = 1;
              }
              break;

            case 1: // collector arm closing
              if (CollectorAngle <= ClosedAngle) {
                CollectorAngle = ClosedAngle;
                if (onGround) {
                  servoCase = 2;
                } else {
                  driveIndex = 0;
                  servoCase = 3;
                }
              } else {
                delayMillis = currentMillis;
                CollectorAngle -= 0.5; // rotational speed of the servo arms movement
              }
              ledcWrite(CollectorServoChannel, degreesToDutyCycle(CollectorAngle));
              break;

            case 2: // collector arm opening
              if (CollectorAngle >= OpenAngle) {
                CollectorAngle = OpenAngle;
                swingingMillis = currentMillis;
                servoCase = 0;
              } else {
                delayMillis = currentMillis;
                CollectorAngle += 0.5; // rotational speed of the servo arms movement
              }
              ledcWrite(CollectorServoChannel, degreesToDutyCycle(CollectorAngle));
              break;

            case 3: // lift scoop
              if (Arm1_Angle >= LiftedAngle1) {
                Arm1_Angle = LiftedAngle1;
                Arm2_Angle = LiftedAngle2;
                delayMillis = currentMillis;
                servoCase = 4;
              } else {
                driveIndex = 0;
                Arm1_Angle += 1; // rotational speed of the servo arms movement
                Arm2_Angle -= 1; // rotational speed of the servo arms movement
              }
              ledcWrite(ArmServo1_Channel, degreesToDutyCycle(Arm1_Angle));// right servo top position
              ledcWrite(ArmServo2_Channel, degreesToDutyCycle(Arm2_Angle));// left servo top position
              break;

            case 4: // drop scoop
              if (Arm1_Angle <= DroppedAngle1) {
                Arm1_Angle = DroppedAngle1;
                Arm2_Angle = DroppedAngle2;
                collectorMillis = currentMillis;
                delayMillis = currentMillis;
                onGround = true;
                servoCase = 0;
                driveIndex = 1;
              } else {
                Arm1_Angle -= 1; // rotational speed of the servo arms movement
                Arm2_Angle += 1; // rotational speed of the servo arms movement
              }
              ledcWrite(ArmServo1_Channel, degreesToDutyCycle(Arm1_Angle));// right servo top position
              ledcWrite(ArmServo2_Channel, degreesToDutyCycle(Arm2_Angle));// left servo top position
              break;
            
            case 5:
              ledcWrite(CollectorServoChannel, degreesToDutyCycle(ClosedAngle));
              ledcWrite(ArmServo1_Channel, degreesToDutyCycle(LiftedAngle1));// right servo top position
              ledcWrite(ArmServo2_Channel, degreesToDutyCycle(LiftedAngle2));// left servo top position
              break;
          }
        }
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
