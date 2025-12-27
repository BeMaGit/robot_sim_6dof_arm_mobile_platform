/*
 * Design A - Smooth Robot Control (ServoEasing + CRSF)
 * Hardware: Arduino Mega 2560 + RadioMaster BR1
 * * DEPENDENCIES:
 * 1. AlfredoCRSF (Manage Libraries -> "AlfredoCRSF")
 * 2. ServoEasing (Manage Libraries -> "ServoEasing")
 */

#include <AlfredoCRSF.h>
#include <ServoEasing.hpp> // Note the .hpp extension for this library
#include <ArduinoJson.h>

// --- Configuration ---

// CRSF Setup (RX1 on Mega is Pin 19)
#define CRSF_BAUDRATE 420000

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

// --- Objects ---
AlfredoCRSF crsf;
ServoEasing sWaist;
ServoEasing sShoulder1;
ServoEasing sShoulder2; // Added secondary shoulder servo object
ServoEasing sElbow;
ServoEasing sWristP;
ServoEasing sWristR;
ServoEasing sGripper;

// --- Variables ---
int chDrive, chTurn;
int chWaist, chShoulder, chElbow, chWristP, chWristR, chGripper;

void setup() {
  Serial.begin(115200);
  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  // 1. Attach Servos with Initial Angles
  // attach(pin, start_angle)
  sWaist.attach(PIN_WAIST, 90);    // 90 = Middle (Facing Back/Driven Wheels)
  sShoulder1.attach(PIN_SHOULDER_1, 10); // Sim -80 -> Servo 10
  sShoulder2.attach(PIN_SHOULDER_2, 170); // 180 - 10 = 170
  sElbow.attach(PIN_ELBOW, 160);    // Sim 160 -> Servo 160
  sWristP.attach(PIN_WRIST_P, 100); // Sim 10 -> Servo 100 (90+10)
  sWristR.attach(PIN_WRIST_R, 180); // Sim 90 -> Servo 180 (90+90)
  sGripper.attach(PIN_GRIPPER, 0);

  // 2. Configure Smoothing
  // EASE_CUBIC_IN_OUT gives a nice "Slow Start -> Fast -> Slow Stop" motion
  sWaist.setEasingType(EASE_CUBIC_IN_OUT);
  sShoulder1.setEasingType(EASE_CUBIC_IN_OUT);
  sShoulder2.setEasingType(EASE_CUBIC_IN_OUT);
  sElbow.setEasingType(EASE_CUBIC_IN_OUT);
  sWristP.setEasingType(EASE_CUBIC_IN_OUT);
  sWristR.setEasingType(EASE_CUBIC_IN_OUT);
  sGripper.setEasingType(EASE_LINEAR); // Grippers usually want linear response

  // 3. Set Speed (Degrees per Second)
  setSpeedForAllServos(ARM_SPEED);
  sGripper.setSpeed(GRIPPER_SPEED); // Gripper can be faster

  // Base Motor Setup
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  
  Serial.println("Robot Initialized with ServoEasing (Dual Shoulder Config).");
}

void loop() {
  // 1. Update Receiver
  crsf.update();

  // 2. Control Loop
  if (crsf.isLinkUp()) {
    
    // --- Read Channels (Adjust mapping to your Radio) ---
    chWaist    = crsf.getChannel(1);
    chShoulder = crsf.getChannel(2);
    chDrive    = crsf.getChannel(3);
    chTurn     = crsf.getChannel(4);
    chElbow    = crsf.getChannel(5);
    chWristP   = crsf.getChannel(6);
    chWristR   = crsf.getChannel(7);
    chGripper  = crsf.getChannel(8);

    // --- Arm Control (Non-Blocking) ---
    // startEaseTo() tells the library to calculate the curve to the new target.
    // It runs in the background using interrupts/timers.
    
    // Waist
    int angleWaist = map(chWaist, 1000, 2000, 0, 180);
    if (sWaist.getCurrentAngle() != angleWaist) sWaist.startEaseTo(angleWaist);

    // Shoulder (Dual Servo Sync)
    int angleShoulder = map(chShoulder, 1000, 2000, 0, 180); // Full range
    int angleShoulder2 = 180 - angleShoulder; // Opposite direction for 2nd servo
    
    if (sShoulder1.getCurrentAngle() != angleShoulder) {
      sShoulder1.startEaseTo(angleShoulder);
      sShoulder2.startEaseTo(angleShoulder2);
    }

    // Elbow
    int angleElbow = map(chElbow, 1000, 2000, 0, 180);
    if (sElbow.getCurrentAngle() != angleElbow) sElbow.startEaseTo(angleElbow);

    // Wrist Pitch
    int angleWristP = map(chWristP, 1000, 2000, 0, 180);
    if (sWristP.getCurrentAngle() != angleWristP) sWristP.startEaseTo(angleWristP);

    // Wrist Roll
    int angleWristR = map(chWristR, 1000, 2000, 0, 180);
    if (sWristR.getCurrentAngle() != angleWristR) sWristR.startEaseTo(angleWristR);

    // Gripper
    int angleGrip = (chGripper > 1500) ? 90 : 0;
    if (sGripper.getCurrentAngle() != angleGrip) sGripper.startEaseTo(angleGrip);

    // --- Base Control (Standard PWM) ---
    controlMobileBase(chDrive, chTurn);

  } else {
    stopBase();
  }
  

  // Note: ServoEasing on AVR (Mega) uses interrupts, so we don't strictly 
  // need to call a update() function in the loop for the servos to move.
  
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

  int leftSpeed = speedFwd + speedTurn;
  int rightSpeed = speedFwd - speedTurn;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftSpeed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightSpeed);
}

void setMotor(int pinPWM, int pinDir, int speed) {
  if (speed > 0) {
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinDir, LOW);
    analogWrite(pinPWM, abs(speed));
  }
}

void stopBase() {
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
}

// Helper to set speed for all arm joints
void setSpeedForAllServos(int speed) {
  sWaist.setSpeed(speed);
  sShoulder1.setSpeed(speed);
  sShoulder2.setSpeed(speed); // Check logic: 180-angle needs same speed
  sElbow.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
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
       setSpeedForAllServos(servos["speed"]);
    }

    if (servos.containsKey("waist"))    sWaist.startEaseTo(servos["waist"]);
    if (servos.containsKey("shoulder")) {
      int angle = servos["shoulder"];
      sShoulder1.startEaseTo(angle);
      sShoulder2.startEaseTo(180 - angle);
    }
    if (servos.containsKey("elbow"))    sElbow.startEaseTo(servos["elbow"]);
    if (servos.containsKey("wristP"))   sWristP.startEaseTo(servos["wristP"]);
    if (servos.containsKey("wristR"))   sWristR.startEaseTo(servos["wristR"]);
    if (servos.containsKey("gripper"))  sGripper.startEaseTo(servos["gripper"]);
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
