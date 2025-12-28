/*
 * Motor Test Program
 * Tests base motors connected to pins 8-11
 * 
 * Hardware Assumption:
 * Left Motor: PWM Pin 8, Direction Pin 9
 * Right Motor: PWM Pin 10, Direction Pin 11
 */

const int PIN_LEFT_PWM  = 9;
const int PIN_LEFT_DIR  = 10;
const int PIN_RIGHT_PWM = 11;
const int PIN_RIGHT_DIR = 12;

// Motor speed for testing (0-255)
const int TEST_SPEED = 150; 

void setup() {
  // Initialize Serial for debug output
  Serial.begin(9600);
  Serial.println("Starting Motor Test...");
  Serial.println("Ensure robot is on a stand or wheels are free to move.");
  delay(2000); // 2 second delay to prep

  // Set pins as outputs
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  
  stopMotors();
}

void loop() {
  Serial.println("Moving FORWARD");
  moveForward(TEST_SPEED);
  delay(2000);
  
  Serial.println("STOP");
  stopMotors();
  delay(1000);
  
  Serial.println("Moving BACKWARD");
  moveBackward(TEST_SPEED);
  delay(2000);
  
  Serial.println("STOP");
  stopMotors();
  delay(1000);
  
  Serial.println("Turning LEFT");
  turnLeft(TEST_SPEED);
  delay(2000);
  
  Serial.println("STOP");
  stopMotors();
  delay(1000);
  
  Serial.println("Turning RIGHT");
  turnRight(TEST_SPEED);
  delay(2000);
  
  Serial.println("STOP");
  stopMotors();
  delay(3000); // Longer pause before restarting
}

// --- Helper Functions ---

void setMotor(int pinPWM, int pinDir, int speed) {
  // Speed ranges from -255 to 255
  // Positive = Forward (or one direction)
  // Negative = Backward
  
  if (speed >= 0) {
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinDir, LOW);
    analogWrite(pinPWM, abs(speed));
  }
}

void moveForward(int speed) {
  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, speed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, speed);
}

void moveBackward(int speed) {
  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, -speed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, -speed);
}

void turnLeft(int speed) {
  // Left motor back, Right motor forward
  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, -speed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, speed);
}

void turnRight(int speed) {
  // Left motor forward, Right motor back
  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, speed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, -speed);
}

void stopMotors() {
  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, 0);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, 0);
}
