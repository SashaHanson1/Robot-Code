#include <ESP32Servo.h>

// Define servo pins
#define FRONT_SERVO_PIN 32
#define ARM_SERVO_PIN 33  // Adjust based on your actual setup

// Define servo positions and movement parameters
#define FRONT_SERVO_POS_90 90
#define FRONT_SERVO_POS_0 0
#define ARM_SERVO_POS_120 120
#define ARM_SERVO_POS_DOWN 0  // Assuming down position as 0 degrees
#define SERVO_MOVE_INTERVAL 20 // Time in milliseconds between servo movements to control speed

// Create servo objects
Servo FrontServo;
Servo ArmServo;

// Variables to manage servo positions and timing
int frontServoTargetPosition = FRONT_SERVO_POS_90; // Target position for front servo
int armServoTargetPosition = ARM_SERVO_POS_DOWN; // Target position for arm servo
unsigned long lastFrontServoMoveTime = 0;
unsigned long lastArmServoMoveTime = 0;
unsigned long frontServoChangeTime = 0;
unsigned long armServoChangeTime = 0;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  FrontServo.attach(FRONT_SERVO_PIN);
  ArmServo.attach(ARM_SERVO_PIN);
  FrontServo.write(frontServoTargetPosition);
  ArmServo.write(armServoTargetPosition);
}

void loop() {
  unsigned long currentMillis = millis();

  // Front servo control
  if (currentMillis - frontServoChangeTime >= 15000) { // Every 15 seconds
    frontServoChangeTime = currentMillis;
    frontServoTargetPosition = (frontServoTargetPosition == FRONT_SERVO_POS_90) ? FRONT_SERVO_POS_0 : FRONT_SERVO_POS_90;
  }

  // Slow movement control for FrontServo
  if (currentMillis - lastFrontServoMoveTime > SERVO_MOVE_INTERVAL) {
    lastFrontServoMoveTime = currentMillis;
    int currentPos = FrontServo.read();
    if (currentPos < frontServoTargetPosition) {
      FrontServo.write(currentPos + 1); // Increment position for slow movement
    } else if (currentPos > frontServoTargetPosition) {
      FrontServo.write(currentPos - 1); // Decrement position for slow movement
    }
  }

  // Arm servo control
  if (FrontServo.read() == FRONT_SERVO_POS_0 && currentMillis - armServoChangeTime >= 30000) {
    armServoChangeTime = currentMillis;
    armServoTargetPosition = (armServoTargetPosition == ARM_SERVO_POS_DOWN) ? ARM_SERVO_POS_120 : ARM_SERVO_POS_DOWN;
  }

  // Slow movement control for ArmServo
  if (currentMillis - lastArmServoMoveTime > SERVO_MOVE_INTERVAL && armServoTargetPosition == ARM_SERVO_POS_120) {
    lastArmServoMoveTime = currentMillis;
    int currentPos = ArmServo.read();
    if (currentPos < armServoTargetPosition) {
      ArmServo.write(currentPos + 1); // Increment position for slow movement
    } else if (currentPos > armServoTargetPosition) {
      ArmServo.write(currentPos - 1); // Decrement position for slow movement
    }
  }
}