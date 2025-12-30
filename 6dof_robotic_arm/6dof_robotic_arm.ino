#include <AlfredoCRSF.h>
// #include <ServoEasing.hpp> // Removed
#include <Servo.h>         // Direct control for all servos
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

// State Variables for Relative Control
float tWaist, tShoulder, tElbow, tWristP, tWristR, tGripper; 

// --- Objects ---
AlfredoCRSF crsf;
Servo sWaist; // CHANGED: Direct Servo
Servo sShoulder1; // CHANGED: Direct Servo
Servo sShoulder2; // CHANGED: Direct Servo
Servo sElbow; 
Servo sWristP; // CHANGED: Direct Servo
Servo sWristR; // CHANGED: Direct Servo
Servo sGripper; // CHANGED: Direct Servo

// --- Variables ---
int chDrive, chTurn;
int chWaist, chShoulder, chElbow, chWristP, chWristR, chGripper;

void setup() {
  Serial.begin(115200);
  Serial2.begin(CRSF_BAUDRATE);
  crsf.begin(Serial2);

  // 1. Attach Servos with Initial Angles
  // attach(pin, start_angle)
  sWaist.attach(PIN_WAIST); sWaist.write(90); tWaist = 90;
  
  // Shoulder Direct Config
  sShoulder1.attach(PIN_SHOULDER_1);
  sShoulder2.attach(PIN_SHOULDER_2);
  tShoulder = 180;
  sShoulder1.write((int)tShoulder);
  sShoulder2.write(180 - (int)tShoulder);

  
  // Elbow Direct Config
  sElbow.attach(PIN_ELBOW); 
  sElbow.write(180);
  tElbow = 180;

  // Wrist & Gripper Direct Config
  sWristP.attach(PIN_WRIST_P); sWristP.write(100); tWristP = 100;
  sWristR.attach(PIN_WRIST_R); sWristR.write(90); tWristR = 90;
  sGripper.attach(PIN_GRIPPER); sGripper.write(45); tGripper = 45;

  // 2. Configure Smoothing
  // Direct Servo control does not use setEasingType.

  // 3. Set Speed (Degrees per Second)
  // Direct Servo control handles speed via SENSITIVITY constants in loop().

  // Base Motor Setup
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  
  Serial.println("Robot Initialized (All Joints using Standard Servo).");
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

    // DEBUG: Arm Joints
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) { // Print 5 times/sec
      lastPrint = millis();
      Serial.print("Elb:"); Serial.print(tElbow);
      Serial.print(" Shou:"); Serial.print(tShoulder);
      Serial.print(" WrP:"); Serial.print(tWristP);
      Serial.print(" Grip:"); Serial.println(tGripper);
    }

    // --- Arm Control (Relative/Rate) ---
    // Update target angles based on joystick deflection
    
    // Waist - Direct Write
    if (abs(chWaist - 1500) > DEADZONE) tWaist += (chWaist - 1500) * WAIST_SENSITIVITY;
    tWaist = constrain(tWaist, 0, 180);
    sWaist.write((int)tWaist);

    // Shoulder - Direct Write
    if (abs(chShoulder - 1500) > DEADZONE) tShoulder += (chShoulder - 1500) * SHOULDER_SENSITIVITY;
    tShoulder = constrain(tShoulder, 0, 180);                                                 
    // Write to both servos, inversely
    sShoulder1.write((int)tShoulder);
    sShoulder2.write(180 - (int)tShoulder);

    // Elbow - Direct Write
    if (abs(chElbow - 1500) > DEADZONE) tElbow += (chElbow - 1500) * ELBOW_SENSITIVITY;
    tElbow = constrain(tElbow, 0, 180);
    sElbow.write((int)tElbow);

    // Wrist Pitch - Direct Write
    if (abs(chWristP - 1500) > DEADZONE) tWristP += (chWristP - 1500) * WRIST_SENSITIVITY;
    tWristP = constrain(tWristP, 0, 180);
    sWristP.write((int)tWristP);

    // Wrist Roll - Direct Write
    if (abs(chWristR - 1500) > DEADZONE) tWristR += (chWristR - 1500) * WRIST_SENSITIVITY;
    tWristR = constrain(tWristR, 0, 180);
    sWristR.write((int)tWristR);

    // Gripper - Direct Write
    if (abs(chGripper - 1500) > DEADZONE) tGripper += (chGripper - 1500) * GRIPPER_SENSITIVITY;
    tGripper = constrain(tGripper, 0, 90); // Gripper usually 0-90
    sGripper.write((int)tGripper);


    // --- Base Control (Standard PWM) ---
    controlMobileBase(chDrive, chTurn);

  } else {
    stopBase();
  }
  

  // Note: Standard Servo library works without explicit update call
  
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
}

// Helper to set speed for all arm joints
void setSpeedForAllServos(int speed) {
  // Direct Servo control; speed handled by sensitivity
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

    if (servos.containsKey("waist"))    { tWaist = servos["waist"].as<int>(); sWaist.write((int)tWaist); }
    if (servos.containsKey("shoulder")) {
      tShoulder = servos["shoulder"].as<int>();
      sShoulder1.write((int)tShoulder); 
      sShoulder2.write(180 - (int)tShoulder);
    }
    if (servos.containsKey("elbow"))    { tElbow = servos["elbow"].as<int>(); sElbow.write((int)tElbow); } 
    if (servos.containsKey("wristP"))   { tWristP = servos["wristP"].as<int>(); sWristP.write((int)tWristP); }
    if (servos.containsKey("wristR"))   { tWristR = servos["wristR"].as<int>(); sWristR.write((int)tWristR); }
    if (servos.containsKey("gripper"))  { tGripper = servos["gripper"].as<int>(); sGripper.write((int)tGripper); }
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
