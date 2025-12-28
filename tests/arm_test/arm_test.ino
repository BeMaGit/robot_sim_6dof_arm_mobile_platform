/*
 * Arm Test Program
 * Tests individual arm joints sequentially.
 * 
 * SPECIAL CONFIGURATION:
 * - SHOULDER Joint is driven by TWO servos in opposite directions.
 *   - Shoulder 1: Pin 3
 *   - Shoulder 2: Pin 4 -> Moves 180 - Shoulder1_Angle
 * 
 * Pinout (Based on your recent edits):
 * 2:  Waist
 * 3:  Shoulder 1 (Left?)
 * 4:  Shoulder 2 (Right?)
 * 5:  Elbow
 * 6:  Wrist Pitch
 * 7:  Wrist Roll
 * 8:  Gripper
 */

#include <ServoEasing.hpp>

// --- Pin Definitions ---
const int PIN_WAIST     = 2;
const int PIN_SHOULDER_1 = 3;
const int PIN_SHOULDER_2 = 4;
const int PIN_ELBOW     = 5;
const int PIN_WRIST_P   = 6;
const int PIN_WRIST_R   = 7;
const int PIN_GRIPPER   = 8;

// --- Servo Objects ---
ServoEasing sWaist;
ServoEasing sShoulder1;
ServoEasing sShoulder2; // Secondary Shoulder Servo
ServoEasing sElbow;
ServoEasing sWristP;
ServoEasing sWristR;
ServoEasing sGripper;

// --- Settings ---
const int TEST_SPEED = 60; // Degrees per second

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Arm Test (Dual Shoulder Version - Full 180)");
  Serial.println("Warning: Ensure arm has free space to move!");
  delay(2000);

  // Attach Servos
  if (sWaist.attach(PIN_WAIST, 90) == INVALID_SERVO) Serial.println("Error attaching Waist");
  
  if (sShoulder1.attach(PIN_SHOULDER_1, 90) == INVALID_SERVO) Serial.println("Error attaching Shoulder 1");
  if (sShoulder2.attach(PIN_SHOULDER_2, 90) == INVALID_SERVO) Serial.println("Error attaching Shoulder 2"); 
  
  if (sElbow.attach(PIN_ELBOW, 90) == INVALID_SERVO) Serial.println("Error attaching Elbow");
  if (sWristP.attach(PIN_WRIST_P, 90) == INVALID_SERVO) Serial.println("Error attaching Wrist P");
  if (sWristR.attach(PIN_WRIST_R, 90) == INVALID_SERVO) Serial.println("Error attaching Wrist R");
  if (sGripper.attach(PIN_GRIPPER, 0) == INVALID_SERVO) Serial.println("Error attaching Gripper");

  // Set Smoothing Type
  sWaist.setEasingType(EASE_CUBIC_IN_OUT);
  sShoulder1.setEasingType(EASE_CUBIC_IN_OUT);
  sShoulder2.setEasingType(EASE_CUBIC_IN_OUT);
  sElbow.setEasingType(EASE_CUBIC_IN_OUT);
  sWristP.setEasingType(EASE_CUBIC_IN_OUT);
  sWristR.setEasingType(EASE_CUBIC_IN_OUT);
  sGripper.setEasingType(EASE_LINEAR);

  // Set Speed
  setSpeedForAll(TEST_SPEED);
  
  Serial.println("Initialization Complete.");
  delay(1000);
}

void loop() {
  
  // 1. Test Waist (Single Servo)
  Serial.println("Testing Waist (0-180)...");
  sWaist.startEaseTo(0);
  while(areAnyServosMoving()) delay(20);
  delay(1000);
  sWaist.startEaseTo(180);
  while(areAnyServosMoving()) delay(20);
  delay(1000);
  sWaist.startEaseTo(90);
  while(areAnyServosMoving()) delay(20);
  delay(1000);

  // 2. Test Shoulder (Dual Servo)
  Serial.println("Testing Shoulder (0-180)...");
  moveShoulder(0);
  delay(2000);
  moveShoulder(180);
  delay(2000);
  moveShoulder(90);
  delay(1000);

  // 3. Test Elbow
  Serial.println("Testing Elbow (0-180)...");
  sElbow.startEaseTo(0);
  while(areAnyServosMoving()) delay(20); 
  delay(1000);
  sElbow.startEaseTo(180);
  while(areAnyServosMoving()) delay(20); 
  delay(1000);
  sElbow.startEaseTo(90);
  while(areAnyServosMoving()) delay(20); 
  delay(1000);

  // 4. Test Wrist Pitch
  Serial.println("Testing Wrist Pitch (0-180)...");
  sWristP.startEaseTo(0);
  while(areAnyServosMoving()) delay(20); 
  delay(500);
  sWristP.startEaseTo(180);
  while(areAnyServosMoving()) delay(20); 
  delay(500);
  sWristP.startEaseTo(90);
  while(areAnyServosMoving()) delay(20); 

  // 5. Test Wrist Roll
  Serial.println("Testing Wrist Roll (0-180)...");
  sWristR.startEaseTo(0);
  while(areAnyServosMoving()) delay(20); 
  delay(500);
  sWristR.startEaseTo(180);
  while(areAnyServosMoving()) delay(20); 
  delay(500);
  sWristR.startEaseTo(90);
  while(areAnyServosMoving()) delay(20); 

  // 6. Test Gripper
  Serial.println("Testing Gripper...");
  sGripper.startEaseTo(90); // Close? (Assuming 90 is close for now, could be 180 if requested)
  while(areAnyServosMoving()) delay(20); 
  delay(1000);
  sGripper.startEaseTo(0);  // Open?
  while(areAnyServosMoving()) delay(20); 
  delay(1000);

  Serial.println("Loop Complete. Pausing...");
  delay(3000);
}

// --- Helper Functions ---

bool areAnyServosMoving() {
  return sWaist.isMoving() || sShoulder1.isMoving() || sShoulder2.isMoving() || 
         sElbow.isMoving() || sWristP.isMoving() || sWristR.isMoving() || sGripper.isMoving();
}

// Moves Main and Secondary Shoulder servos in synchronization
void moveShoulder(int angle) {
  // Constrain just in case
  angle = constrain(angle, 0, 180);
  
  // Calculate opposite angle for the reversed servo
  int angleReversed = 180 - angle;
  
  Serial.print("Shoulder Target: "); Serial.print(angle);
  Serial.print(" | Shoulder 2 Target: "); Serial.println(angleReversed);
  
  sShoulder1.startEaseTo(angle);
  sShoulder2.startEaseTo(angleReversed);
  
  while(areAnyServosMoving()) {
    delay(20);
  }
}

void setSpeedForAll(int speed) {
  sWaist.setSpeed(speed);
  sShoulder1.setSpeed(speed);
  sShoulder2.setSpeed(speed);
  sElbow.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
  sGripper.setSpeed(speed * 2); 
}
