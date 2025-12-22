/*
 * Design A - 6 DoF Robot Arm + Mobile Base Controller
 * Hardware: Arduino Mega 2560 + RadioMaster BR1 (via CRSF)
 * * DEPENDENCY: Install "AlfredoCRSF" library via Library Manager
 */

#include <AlfredoCRSF.h>
#include <Servo.h>

// --- Configuration ---

// 1. Serial Setup for CRSF
// On Mega, we use Serial1 (RX1 on Pin 19)
#define CRSF_BAUDRATE 420000

// 2. Pin Definitions (Adjust to your wiring)
// Arm Servos
const int PIN_WAIST     = 2;
const int PIN_SHOULDER  = 3;
const int PIN_ELBOW     = 4;
const int PIN_WRIST_P   = 5;
const int PIN_WRIST_R   = 6;
const int PIN_GRIPPER   = 7;

// Base Motors (assuming L298N driver or similar)
const int PIN_LEFT_PWM  = 8;
const int PIN_LEFT_DIR  = 9;
const int PIN_RIGHT_PWM = 10;
const int PIN_RIGHT_DIR = 11;

// --- Objects ---
AlfredoCRSF crsf;
Servo sWaist, sShoulder, sElbow, sWristP, sWristR, sGripper;

// --- Variables ---
// Channel storage (ELRS uses 1000-2000 range usually)
int chDrive, chTurn;
int chWaist, chShoulder, chElbow, chWristP, chWristR, chGripper;

void setup() {
  // Debug Serial
  Serial.begin(115200);
  Serial.println("Initializing Design A Robot Controller...");

  // CRSF Serial (Receiver connected to Serial1)
  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  // Attach Arm Servos
  sWaist.attach(PIN_WAIST);
  sShoulder.attach(PIN_SHOULDER);
  sElbow.attach(PIN_ELBOW);
  sWristP.attach(PIN_WRIST_P);
  sWristR.attach(PIN_WRIST_R);
  sGripper.attach(PIN_GRIPPER);

  // Setup Base Motor Pins
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);

  // Set Default Positions (Home)
  moveArmHome();
}

void loop() {
  // 1. Read Receiver Data
  crsf.update();

  // 2. Safety Check: Only move if link is active
  if (crsf.isLinkUp()) {
    
    // --- READ CHANNELS ---
    // Mapping specific to your Transmitter setup (e.g., Mode 2)
    // Adjust channel numbers (1-16) based on your Radio Mixer
    
    // Mobile Base (Left Stick)
    chDrive = crsf.getChannel(3); // Throttle (usually Ch3)
    chTurn  = crsf.getChannel(4); // Rudder/Yaw (usually Ch4)

    // Arm Joints (Right Stick + Pots/Sliders)
    chWaist    = crsf.getChannel(1); // Aileron (Right Stick X)
    chShoulder = crsf.getChannel(2); // Elevator (Right Stick Y)
    
    // Aux Channels for Upper Arm (Pots/Switches)
    chElbow    = crsf.getChannel(5); // Pot 1 / Switch A
    chWristP   = crsf.getChannel(6); // Pot 2
    chWristR   = crsf.getChannel(7); // Slider
    chGripper  = crsf.getChannel(8); // Switch B

    // --- CONTROL LOGIC ---
    
    controlMobileBase(chDrive, chTurn);
    controlArm();

  } else {
    // Failsafe: Stop everything if signal lost
    stopBase();
  }
}

// --- Helper Functions ---

void controlMobileBase(int throttle, int steering) {
  // Convert CRSF (1000-2000) to range (-255 to 255)
  int speedFwd = map(throttle, 1000, 2000, -255, 255);
  int speedTurn = map(steering, 1000, 2000, -255, 255);

  // Deadzone to prevent drift
  if (abs(speedFwd) < 20) speedFwd = 0;
  if (abs(speedTurn) < 20) speedTurn = 0;

  // Arcade Drive Mixing
  int leftSpeed = speedFwd + speedTurn;
  int rightSpeed = speedFwd - speedTurn;

  // Constrain to PWM limits
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftSpeed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightSpeed);
}

void setMotor(int pinPWM, int pinDir, int speed) {
  if (speed > 0) {
    digitalWrite(pinDir, HIGH); // Forward
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinDir, LOW);  // Backward
    analogWrite(pinPWM, abs(speed));
  }
}

void controlArm() {
  // Direct mapping example.
  // Ideally, you would use "Incremental" control for smoother movement,
  // but this "Absolute" mapping is simpler for testing.
  
  // Map CRSF (1000-2000us) to Servo Angles (0-180 deg)
  // Note: Adjust min/max angles to prevent mechanical crashing!
  
  sWaist.write(map(chWaist, 1000, 2000, 0, 180));
  sShoulder.write(map(chShoulder, 1000, 2000, 45, 135)); // Restricted range example
  sElbow.write(map(chElbow, 1000, 2000, 0, 180));
  sWristP.write(map(chWristP, 1000, 2000, 0, 180));
  sWristR.write(map(chWristR, 1000, 2000, 0, 180));
  
  // Gripper usually just Open/Close
  if (chGripper > 1500) {
    sGripper.write(90); // Closed
  } else {
    sGripper.write(0);  // Open
  }
}

void moveArmHome() {
  sWaist.write(90);
  sShoulder.write(90);
  sElbow.write(90);
  sWristP.write(90);
  sWristR.write(90);
  sGripper.write(0);
  stopBase();
}

void stopBase() {
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
}