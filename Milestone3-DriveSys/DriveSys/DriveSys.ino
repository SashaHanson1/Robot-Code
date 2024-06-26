/*
 MSE 2202 MSEBot Lab 4 - Exercise: Kyle V.
 Language: Arduino
 Authors: Eugen Porter and Michael Naish
 
 Rev 1 - Initial version 2016
 Rev 2 - Update for MSEduino V0.2
 ...
 Rev 4 - revisit for MSEduino V4.2 2023
 Rev 5 - revisit for MSEduino V5.1 2024

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
    Modify the MSEbot basecode so that after pushing PB1, your MSEbot 
    autonomously follows a path to find and collect 
    a ping pong ball irradiated with an IR signal.

    Each block in the path is a standard 8.5" ⨉ 11" sheet of paper. 
    To keep the paper from catching the Chicago bolts on the bottom 
    of the MSEbot, the sheets should be taped together along the full 
    edge length. The stages of the task are as follows:

      1. Drive forward across the first two sheets of paper
      2. Turn 90º counterclockwise
      3. Drive forward across the third sheet of paper
      4. Stop forward motion and use the IR detector in combination with adjustments of the arm height and/or rotation of the chassis to localize the ping pong ball
      5. Drive forward and grab ping pong ball with the claw
      6. Return to start position, drop the ball, and leave drive mode.

    At all times, the robot must stay within the sheets of paper. 
    If all goes well, your MSEbot should stop in roughly the same 
    place that it started from with the ping pong ball within the claw, 
    before dropping it.
*/

// Uncomment keywords to enable debugging output
//#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Function declarations
void Indicator();  // for mode/heartbeat on Smart LED
void setTarget(int dir, double distance);
void servoReset();

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
#define SHOULDER_SERVO 41      // GPIO41 pin 34 (J41) Servo 1
#define CLAW_SERVO 42          // GPIO42 pin 35 (J42) Servo 2

// Constants
const int cDisplayUpdate = 100;           // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                    // bit resolution for PWM
const int cMinPWM = 150;                  // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;  // PWM value for maximum speed

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot.
//            You will have to experiment to determine appropriate values.

const int cClawServoOpen = 1001;    // Value for open position of claw
const int cClawServoClosed = 2150;  // Value for closed position of claw
const int cArmServoUp = 1000;       // Value for shoulder of arm fully up
const int cArmServoDown = 2199;     // Value for shoulder of arm fully down
const int cRightAdjust = 0;
const int cLeftAdjust = 2;
const long cCountsRev = 1096;
const double distancePerRev = 13.8;  //circumference of the wheels in cm


const int pi = 3.14159;                                                        // pi is used to calculate circumference of a wheel based on its diameter
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
unsigned int potClawSetpoint;         // desired position of claw servo read from pot
unsigned int potShoulderSetpoint;     // desired position of shoulder servo read from pot
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
  Bot.servoBegin("S1", CLAW_SERVO);                                                // set up claw servo
  Bot.servoBegin("S2", SHOULDER_SERVO);                                            // set up shoulder servo
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
}

void loop() {
  long pos[] = { 0, 0 };  // current motor positions
  int pot = 0;            // raw ADC value from pot

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
          robotModeIndex = robotModeIndex & 7;  // keep mode index between 0 and 7
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
        if (timeUp3sec) {  // pause for 3 sec before running case 1 code
          // Read pot to update drive motor speed
          pot = analogRead(POT_R1);
          leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
          rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cRightAdjust;
          // leftDriveSpeed = cMaxPWM - cLeftAdjust;
          // rightDriveSpeed = cMaxPWM - cRightAdjust;
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
            Serial.print(F("Left Encoder count = "));
            Serial.print(LeftEncoder.lRawEncoderCount);
            Serial.print(F("  Right Encoder count = "));
            Serial.print(RightEncoder.lRawEncoderCount);
            Serial.print("\n");

            Serial.print("target: ");
            Serial.println(target);
          }
#endif
          if (motorsEnabled) {          // run motors only if enabled
            if (timeUp2sec) {           // update drive state after 2 seconds
              switch (driveIndex) {     // cycle through drive states
                case 0: // stop robot
                  Bot.Stop("D1");
                  driveIndex = 1;
                  break;

                case 1:
                  Bot.Forward("D1", rightDriveSpeed, leftDriveSpeed);
                  if (abs(LeftEncoder.lRawEncoderCount) >= cDist * 50) {
                    LeftEncoder.clearEncoder();
                    RightEncoder.clearEncoder();
                    driveIndex = 2;
                  }
                  break;

                case 2:
                  Bot.Right("D1", rightDriveSpeed, leftDriveSpeed);
                  if (abs(LeftEncoder.lRawEncoderCount) >= cDist * 3.65 * pi) {
                    LeftEncoder.clearEncoder();
                    RightEncoder.clearEncoder();
                    if (pauseTime == 4) {
                      driveIndex = 0;
                    } else {
                      driveIndex = 1;
                    }
                  }
                  pauseTime++;
                  break;
              }
            }
          } else {  // stop when motors are disabled
            Bot.Stop("D1");
          }
          break;
        }

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
