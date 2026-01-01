#include <AlfredoCRSF.h>
#include <ServoEasing.hpp> 
// #include <Servo.h>         // Direct control for all servos
#include <ArduinoJson.h>

// --- Configuration ---

// CRSF Setup (RX2 on Mega is Pin 17, TX2 is Pin 16)
#define CRSF_BAUDRATE 115200 // Must match Receiver Output Baud Rate

// Pin Definitions
const int PIN_WAIST      = 2;
const int PIN_SHOULDER_1 = 3;
const int PIN_SHOULDER_2 = 4; // Secondary Shoulder Servo (Opposite Direction)
const int PIN_ELBOW      = 5;
const int PIN_WRIST_P    = 6;
const int PIN_WRIST_R    = 7;
const int PIN_GRIPPER    = 8;

// Base Motor Pins
const int PIN_LEFT_PWM   = 9;
const int PIN_LEFT_DIR   = 10;
const int PIN_RIGHT_PWM  = 11;
const int PIN_RIGHT_DIR  = 12;

// --- Tuning ---
// Speed is in Degrees per Second. 
// Lower = Smoother/Heavier feel. Higher = Snappier.
const int ARM_SPEED = 90;      
const int GRIPPER_SPEED = 200; 
const float SENSITIVITY = 0.005; // 0.005 * 500 = 2.5 deg/loop (Fast)
const float WAIST_SENSITIVITY = 0.0005; // Slower for heavy base
const float ELBOW_SENSITIVITY = 0.0001; // Slower for direct Servo control
const float SHOULDER_SENSITIVITY = 0.0002; // Slower for heavy shoulder
const float WRIST_SENSITIVITY = 0.001; // Moderate for wrist
const float GRIPPER_SENSITIVITY = 0.001; // Moderate for gripper
const int DEADZONE = 30;

// Startup & Home Configuration (PWM Values in Microseconds)
const int STARTUP_SPEED = 20; // Deg/Sec for startup
const int WAIST_HOME_PWM    = 1500; // ~90 deg
const int SHOULDER_HOME_PWM = 2400; // ~180 deg
const int ELBOW_HOME_PWM    = 2400; // ~180 deg
const int WRIST_P_HOME_PWM  = 1500; // ~90 deg (Middle)
const int WRIST_R_HOME_PWM  = 1500; // ~90 deg
const int GRIPPER_HOME_PWM  = 1000; // ~45 deg

// State Variables for Relative Control
float tWaist, tShoulder, tElbow, tWristP, tWristR, tGripper; 

// --- IK Configuration & State ---
const float L1 = 200.0; // Shoulder to Elbow (mm)
const float L2 = 140.0; // Elbow to Wrist (mm)
bool ikMode = false;
float currentX = 100.0; // Initial guess, will be synced
float currentY = 100.0;
float globalWristAngle = 0.0; // Angle relative to horizon

// --- Objects ---
AlfredoCRSF crsf;
ServoEasing sWaist; 
ServoEasing sShoulder1; 
ServoEasing sShoulder2; 
ServoEasing sElbow; 
ServoEasing sWristP; 
ServoEasing sWristR; 
ServoEasing sGripper;

// --- Variables ---
int chDrive, chTurn;
int chWaist, chShoulder, chElbow, chWristP, chWristR, chGripper;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// Conversion Helper
long map_long(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int pwmToAngle(int pwm) {
   // Standard Servo Range 544..2400 -> 0..180
   // Constrain to be safe
   int val = constrain(pwm, 544, 2400);
   return map_long(val, 544, 2400, 0, 180);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(CRSF_BAUDRATE);
  crsf.begin(Serial2);

  Serial.println("Attach Servos...");

  // 1. Attach Servos - Explicitly define Min/Max Pulse Widths (Standard: 544-2400)
  // This ensures that 0-180 degrees maps EXACTLY to 544-2400 microseconds.
  // Using generic attach() might default to different values on some boards.
  
  sWaist.attach(PIN_WAIST, 90, 544, 2400); 
  
  sShoulder1.attach(PIN_SHOULDER_1, 180, 544, 2400); 
  sShoulder2.attach(PIN_SHOULDER_2, 0, 544, 2400);
  
  sElbow.attach(PIN_ELBOW, 180, 544, 2400); 
  
  sWristP.attach(PIN_WRIST_P, 90, 544, 2400);
  sWristR.attach(PIN_WRIST_R, 90, 544, 2400);
  sGripper.attach(PIN_GRIPPER, 90, 544, 2400);

  // 2. Slow Startup Sequence
  Serial.println("Starting Homing Sequence (PWM -> Angle)...");
  
  // Set Slow Speed
  setSpeedForAllServos(STARTUP_SPEED);
  
  // Calculate Targets from PWM
  // Since we attached with 544-2400, this mapping will be accurate.
  tWaist    = pwmToAngle(WAIST_HOME_PWM);
  tShoulder = pwmToAngle(SHOULDER_HOME_PWM);
  tElbow    = pwmToAngle(ELBOW_HOME_PWM);
  tWristP   = pwmToAngle(WRIST_P_HOME_PWM);
  tWristR   = pwmToAngle(WRIST_R_HOME_PWM);
  tGripper  = pwmToAngle(GRIPPER_HOME_PWM);

  // Command Easing
  // Note: ServoEasing library generally requires Degrees for easing commands.
  sWaist.startEaseTo((int)tWaist);
  
  // Shoulder logic: S1 normal, S2 inverse
  sShoulder1.startEaseTo((int)tShoulder);
  sShoulder2.startEaseTo(180 - (int)tShoulder);
  
  sElbow.startEaseTo((int)tElbow);
  sWristP.startEaseTo((int)tWristP);
  sWristR.startEaseTo((int)tWristR);
  sGripper.startEaseTo((int)tGripper);
  
  // Blocking wait for startup move to complete
  // Checking individual Servos as static check helper might be missing/named differently
  while(sWaist.isMoving() || sShoulder1.isMoving() || sShoulder2.isMoving() || 
        sElbow.isMoving() || sWristP.isMoving() || sWristR.isMoving() || sGripper.isMoving()) {
    delay(20); 
  }
  
  Serial.println("Homing Complete.");

  // 3. Restore Operational Speed
  setSpeedForAllServos(ARM_SPEED); // 90 by default

  // Base Motor Setup
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  
  Serial.println("Robot Ready.");
}

void loop() {
  // 1. Update Receiver
  crsf.update();

  // 2. Control Loop
  if (crsf.isLinkUp()) {
    
    // --- Read Channels (Adjust mapping to your Radio) ---
    chWaist    = crsf.getChannel(4); // Mapped to Ch4 
    chShoulder = crsf.getChannel(2);
    chDrive    = crsf.getChannel(3);
    chTurn     = crsf.getChannel(1); // Mapped to Ch1 
    chElbow    = crsf.getChannel(6); 
    chWristP   = crsf.getChannel(7); 
    chWristR   = crsf.getChannel(8);
    chGripper  = crsf.getChannel(9);

    // DEBUG: JSON Output
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) { // Print 10 times/sec (Smoother UI)
      lastPrint = millis();
      printStatusJSON();
    }

    // --- Mode Switch ---
    int chMode = crsf.getChannel(5); // Aux 1
    if (chMode > 1500 && !ikMode) {
       ikMode = true;
       syncFK();
       Serial.println("Wrist Lock Active");
    } else if (chMode <= 1500 && ikMode) {
       ikMode = false;
       Serial.println("Manual Mode Active");
    }

    // --- Arm Control ---
    
    // Always Allow Waist, Gripper, Wrist Roll
    // Waist
    if (abs(chWaist - 1500) > DEADZONE) tWaist += (chWaist - 1500) * WAIST_SENSITIVITY;
    tWaist = constrain(tWaist, 0, 180);
    sWaist.write((int)tWaist);

    // Wrist Roll
    if (abs(chWristR - 1500) > DEADZONE) tWristR += (chWristR - 1500) * WRIST_SENSITIVITY;
    tWristR = constrain(tWristR, 0, 180);
    sWristR.write((int)tWristR);

    // Gripper
    if (abs(chGripper - 1500) > DEADZONE) tGripper += (chGripper - 1500) * GRIPPER_SENSITIVITY;
    tGripper = constrain(tGripper, 0, 90);
    sGripper.write((int)tGripper);
    
    // Manual Shoulder & Elbow (Always Active now)
    // Shoulder
    if (abs(chShoulder - 1500) > DEADZONE) tShoulder += (chShoulder - 1500) * SHOULDER_SENSITIVITY;
    tShoulder = constrain(tShoulder, 0, 180);                                                 
    sShoulder1.write((int)tShoulder);
    sShoulder2.write(180 - (int)tShoulder);

    // Elbow
    if (abs(chElbow - 1500) > DEADZONE) tElbow += (chElbow - 1500) * ELBOW_SENSITIVITY;
    tElbow = constrain(tElbow, 0, 180);
    sElbow.write((int)tElbow);

    if (ikMode) {
       // activeWristCompensation
       
       // Allow Manual Adjustment of Global Pitch (Wrist)
       // If user moves Wrist Stick, update globalWristAngle
       if (abs(chWristP - 1500) > DEADZONE) {
           // Map input to speed or angle change? 
           // Standard manual control drives angle directly proportional to stick (if we used position control)
           // But here we want to adjust the OFFSET.
           // However, existing manual logic acts as "Angle Control" (tWristP += ...)
           // Let's do similar: Update globalWristAngle based on stick input.
           globalWristAngle += (chWristP - 1500) * WRIST_SENSITIVITY * 0.001; // Scale down for smooth adjustment
           // globalWristAngle is in Radians? No, wait. 
           // In updateIKMode, we used radians. 
           // Let's check Global Variable definition.
           // const float toRad = PI/180.0;
           // globalWristAngle = 0; // It's a float. 
           // Let's assume it is in RADIANS because we used: float qWrist = globalWristAngle - (q1 + q2);
           // And q1/q2 are radians.
           // So (chWristP - 1500) is large (~500). Sensitivity is ~0.05?
           // We need small radian increments. 
           // Let's try:
           float sensitivityRad = 0.00001; 
           globalWristAngle += (chWristP - 1500) * sensitivityRad;
       }

       // Calculate required Wrist Pitch to maintain global angle
       // qWrist = Global - (q1 - q2)
       // User Code was: 90 - (Global - (q1 + q2)) = 90 - Global + q1 + q2.
       // User says Elbow (q2) is Wrong Direction -> Needs to be -q2.
       // New: 90 - (Global - (q1 - q2)) = 90 - Global + q1 - q2.
       float q1 = servoToRadS(tShoulder); 
       float q2 = servoToRadE(tElbow);
       float qWrist = globalWristAngle - (q1 - q2);
       
       // Map to Servo Angle
       // If previous "Standard" direction (deg + 90) was "Wrong Direction" (Skyward),
       // We need to invert the Servo Output.
       // Standard: 90 + deg. Inverted: 90 - deg. (Or 180 - (90+deg))
       tWristP = 90 - degrees(qWrist);
       tWristP = constrain(tWristP, 0, 180);
       sWristP.write((int)tWristP);
       
       // Sync Cartesian State just in case we switch back to full IK later or for logs
       currentX = L1 * cos(q1) + L2 * cos(q1 + q2);
       currentY = L1 * sin(q1) + L2 * sin(q1 + q2);
       
    } else {
       // Manual Wrist Pitch
       if (abs(chWristP - 1500) > DEADZONE) tWristP += (chWristP - 1500) * WRIST_SENSITIVITY;
       tWristP = constrain(tWristP, 0, 180);
       sWristP.write((int)tWristP);
    }
    
    // --- Base Control (Standard PWM) ---
    // Enable Base Control always (User undid "IK Mode")
    controlMobileBase(chDrive, chTurn);
  } else {
    stopBase();
  }

  // 3. JSON Control (USB)
  if (Serial.available()) {
    processJSON();
  }
}

// --- Helper Functions ---

void controlMobileBase(int throttle, int steering) {
  int speedFwd = map(throttle, 1000, 2000, -255, 255);
  int speedTurn = map(steering, 1000, 2000, -255, 255);

  if (abs(speedFwd) < 20) speedFwd = 0;
  if (abs(speedTurn) < 20) speedTurn = 0;

  int leftSpeed = speedFwd - speedTurn;
  int rightSpeed = speedFwd + speedTurn;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftSpeed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightSpeed);
}

void setMotor(int pinPWM, int pinDir, int speed) {
  if (speed > 0) {
    // Forward (Stick Up) - Normal Logic
    // Based on testing: DIR=LOW is Forward, PWM 255 is Fast.
    digitalWrite(pinDir, LOW);
    analogWrite(pinPWM, speed); 
  } else {
    // Backward (Stick Down) - Inverted Logic
    // Based on testing: DIR=HIGH is Backward, PWM 0 is Fast (255 is Stop).
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPWM, 255 - abs(speed));
  }
}

void stopBase() {
  // Safe Stop: DIR=HIGH (Inverted) + PWM 255 (Stop)
  digitalWrite(PIN_LEFT_DIR, HIGH);
  digitalWrite(PIN_RIGHT_DIR, HIGH);
  analogWrite(PIN_LEFT_PWM, 255);
  analogWrite(PIN_RIGHT_PWM, 255);
  
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

// Helper to set speed for all arm joints
void setSpeedForAllServos(int speed) {
  // ServoEasing uses degrees/sec
  sWaist.setSpeed(speed);
  sShoulder1.setSpeed(speed);
  sShoulder2.setSpeed(speed);
  sElbow.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
  sGripper.setSpeed(speed);
}

void printStatusJSON() {
  StaticJsonDocument<512> doc;

  JsonObject servos = doc.createNestedObject("servos");
  servos["waist"] = (int)tWaist;
  servos["shoulder"] = (int)tShoulder;
  servos["elbow"] = (int)tElbow;
  servos["wristP"] = (int)tWristP;
  servos["wristR"] = (int)tWristR;
  servos["gripper"] = (int)tGripper;
  // Speed is static/const in this logic, but we can include it if needed.
  // servos["speed"] = 60; 

  JsonObject motors = doc.createNestedObject("motors");
  motors["left"] = currentLeftSpeed;
  motors["right"] = currentRightSpeed;
  motors["duration"] = 0;

  serializeJson(doc, Serial);
  Serial.println();
}

void processJSON() {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, Serial);

  if (error) {
    // If it's just a newline or garbage, ignore or print error
    // Serial.print(F("deserializeJson() failed: ")); 
    // Serial.println(error.f_str());
    return;
  }

  // --- Process Servos ---
  if (doc.containsKey("servos")) {
    JsonObject servos = doc["servos"];
    
    // Check for speed first
    if (servos.containsKey("speed")) {
       setSpeedForAllServos(servos["speed"].as<int>());
    }

    // Use startEaseTo for smooth transitions (non-blocking)
    if (servos.containsKey("waist"))    { tWaist = servos["waist"].as<int>(); sWaist.startEaseTo((int)tWaist); }
    if (servos.containsKey("shoulder")) {
      tShoulder = servos["shoulder"].as<int>();
      sShoulder1.startEaseTo((int)tShoulder); 
      sShoulder2.startEaseTo(180 - (int)tShoulder);
    }
    if (servos.containsKey("elbow"))    { tElbow = servos["elbow"].as<int>(); sElbow.startEaseTo((int)tElbow); } 
    if (servos.containsKey("wristP"))   { tWristP = servos["wristP"].as<int>(); sWristP.startEaseTo((int)tWristP); }
    if (servos.containsKey("wristR"))   { tWristR = servos["wristR"].as<int>(); sWristR.startEaseTo((int)tWristR); }
    if (servos.containsKey("gripper"))  { tGripper = servos["gripper"].as<int>(); sGripper.startEaseTo((int)tGripper); }
  }

  // --- Process Motors ---
  if (doc.containsKey("motors")) {
    JsonObject motors = doc["motors"];
    int left = motors["left"] | 0;
    int right = motors["right"] | 0;
    int duration = motors["duration"] | 0;

    setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, left);
    setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, right);

    if (duration > 0) {
      delay(duration); // Blocking delay for motor duration if specified
      setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, 0);
      setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, 0);
    }
  }
}

// --- IK Functions ---

// Convert Servo Angle (0-180) to Geometric Angle (radians)
float servoToRadS(float angle) {
  // Shoulder: 180 is Up/Back? 
  // Code says 180 is home. 2400us.
  // Let's assume 180 is vertical up for now to match "elbow up" logic
  // If 180 is up (90 deg geom), and 90 is forward (0 deg geom).
  // Then angle_geom = angle_servo - 90.
  // But let's stick to the previous plan: standard servo 0..180
  return radians(angle); 
}

float servoToRadE(float angle) {
  // If 180 is straight out relative to shoulder link?
  return radians(angle - 180);
}

// Convert Geometric Angle (radians) to Servo Angle (0-180)
float radToServoS(float angle) {
  return degrees(angle);
}

float radToServoE(float angle) {
  return degrees(angle) + 180;
}

void syncFK() {
  // Sync state from current servo positions
  // Note: Only approximations if servo isn't calibrated perfectly to math model
  float q1 = radians(tShoulder); 
  float q2 = radians(tElbow - 180); 
  
  // Forward Kinematics
  // Assumes q1=0 is Horizontal Forward, q2=0 is Straight
  // x = l1 c1 + l2 c(1+2)
  // y = l1 s1 + l2 s(1+2)
  currentX = L1 * cos(q1) + L2 * cos(q1 + q2);
  currentY = L1 * sin(q1) + L2 * sin(q1 + q2);
  
  // Wrist Pitch relative to horizon
  // global = q1 + q2 + qWrist
  // If tWristP=90 is straight with arm -> qWrist=0
  float qWrist = radians(tWristP - 90);
  globalWristAngle = q1 + q2 + qWrist;
}

void updateIKMode(int inputX, int inputY) {
  // Input Deadzone and Scaling
  float dX = 0;
  float dY = 0;
  
  // Ch1 (Turn) -> X (Reach). Right is Positive (Forward?)
  // Ch2 (Pitch) -> Y (Height). Up is Positive.
  
  if (abs(inputX - 1500) > DEADZONE) dX = (inputX - 1500) * 0.005; // Tunable Speed
  if (abs(inputY - 1500) > DEADZONE) dY = (inputY - 1500) * 0.005;
  
  if (dX == 0 && dY == 0) return; 
  
  // Proposed New Position
  float nextX = currentX + dX;
  float nextY = currentY + dY;
  
  // Check Reachability (Simple Radius Check)
  float distSq = nextX*nextX + nextY*nextY;
  float maxReach = (L1 + L2) * 0.99; 
  if (distSq > maxReach * maxReach) {
     return; // Limit reached
  }
  
  // Solve IK (2-Link Planar)
  // c2 = (x^2 + y^2 - l1^2 - l2^2) / (2 l1 l2)
  float c2 = (distSq - L1*L1 - L2*L2) / (2 * L1 * L2);
  
  if (c2 < -1.0 || c2 > 1.0) return; // Unreachable
  
  // q2 (Elbow Angle). 
  // Using -acos for "Elbow Up" if Y is positive up?
  // Let's assume Elbow Up configuration. 
  // If q2 is negative, it bends "up/back" relative to extended line?
  // Depends on frame.
  float q2 = -acos(c2); 
  
  // q1 (Shoulder Angle)
  // theta1 = atan2(y, x) - atan2(k2, k1)
  float k1 = L1 + L2 * c2; // L1 + L2 cos(q2)
  float k2 = L2 * sin(q2);
  float q1 = atan2(nextY, nextX) - atan2(k2, k1);
  
  // Servo Constraints
  float sAng = degrees(q1); 
  float eAng = degrees(q2) + 180; 
  
  if (sAng < 0 || sAng > 180 || eAng < 0 || eAng > 180) return; 
  
  // Commit
  currentX = nextX;
  currentY = nextY;
  
  tShoulder = sAng;
  tElbow = eAng;
  
  // Wrist Compensation
  // qWrist = Global - (q1 + q2)
  // tWrist = deg(qWrist) + 90
  float qWrist = globalWristAngle - (q1 + q2);
  tWristP = degrees(qWrist) + 90;
  tWristP = constrain(tWristP, 0, 180);
  
  // Apply to Servos
  sShoulder1.write((int)tShoulder);
  sShoulder2.write(180 - (int)tShoulder);
  sElbow.write((int)tElbow);
  sWristP.write((int)tWristP);
}

