/*
 * Platform CRSF Test (BetaFPV ELRS Micro RX)
 * 
 * Purpose: Test mobile platform control using a BetaFPV ELRS Micro RX (or compatible) via CRSF protocol.
 * 
 * Hardware: 
 * - Arduino Mega 2560
 * - Receiver: BetaFPV ELRS Micro RX (configured for CRSF output)
 * - Motor Driver (L298N or similar) connected to Pins 9, 10, 11, 12
 * 
 * WIRING:
 * - Receiver TX  -> Arduino RX1 (Pin 19)
 * - Receiver RX  -> Arduino TX1 (Pin 18)
 * - Receiver 5V  -> Arduino 5V
 * - Receiver GND -> Arduino GND
 * 
 * DEPENDENCIES:
 * - "AlfredoCRSF" Library (Search in Library Manager)
 */

#include <AlfredoCRSF.h>

// --- Configuration ---

// CRSF Setup
// SWITCHING TO SERIAL2 (Pins 16/17 on Mega) for better isolation
// Receiver TX (Ch2) -> Mega RX2 (Pin 17)
// Receiver RX (Ch3) -> Mega TX2 (Pin 16)
AlfredoCRSF crsf;

// Debug Mode: Set to false to parse CRSF data
bool debugRaw = false; 

// Motor Driver Pins
const int PIN_LEFT_PWM   = 9;
const int PIN_LEFT_DIR   = 10;
const int PIN_RIGHT_PWM  = 11;
const int PIN_RIGHT_DIR  = 12;

// Channel Mapping (ELRS/CRSF Default usually AETR: 1=A, 2=E, 3=T, 4=R)
// NOTE: AlfredoCRSF uses 1-based indexing for getChannel() usually.
// Let's verify standard mapping:
// Channel 1: Roll (Aileron)
// Channel 2: Pitch (Elevator)
// Channel 3: Throttle
// Channel 4: Yaw (Rudder)
// We will use Throttle (Ch3) and Yaw/Steering (Ch4 or Ch1 depending on preference).
// Let's use Throttle (Ch3) and Steering (Ch1 - Right Stick X or Ch4 - Left Stick X).
// Typicall arcade drive uses Left Stick Y (Throttle) and Right Stick X (Steering).
const int CH_THROTTLE = 3; 
const int CH_STEERING = 1; // 1 = Aileron (Right stick X), 4 = Rudder (Left stick X)

void setup() {
  Serial.begin(115200);
  Serial.println("Platform CRSF Test (ELRS) Starting...");
  Serial.println("Ensure RECEIVER is connected to Serial2 (Pins 16/17).");
  Serial.println("Rx Ch2 -> Pin 17 | Rx Ch3 -> Pin 16");

  // CRSF Start
  // Serial2 needs to be passed to the library. 
  // IMPORTANT: We use 115200 baud because Arduino Mega cannot accurately generate 416666 baud.
  // You MUST configure your ELRS Receiver output to 115200 baud via Web UI / Lua.
  Serial2.begin(115200); 
  if (!debugRaw) {
    crsf.begin(Serial2);
  } else {
    Serial.println("DEBUG MODE: Printing raw bytes from Serial2...");
  }

  // Motor Pins
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
}

void loop() {
  if (debugRaw) {
    if (Serial2.available()) {
      Serial.print(Serial2.read(), HEX);
      Serial.print(" ");
    }
    return;
  }

  // Update CRSF
  crsf.update();

  if (crsf.isLinkUp()) {
    // Read Channels (1000 - 2000 range usually)
    int throttle_raw = crsf.getChannel(CH_THROTTLE);
    int steering_raw = crsf.getChannel(CH_STEERING);

    // Map to Motor Speed (-255 to 255)
    // CRSF standard range is 988 to 2012. Center 1500.
    int throttle = map(throttle_raw, 1000, 2000, -255, 255);
    int steering = map(steering_raw, 1000, 2000, -255, 255);

    // Initial Deadband
    if (abs(throttle) < 20) throttle = 0;
    if (abs(steering) < 20) steering = 0;

    // Arcade Drive Mixing
    int leftSpeed = throttle + steering;
    int rightSpeed = throttle - steering;

    // Constrain
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Output to Motors
    setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftSpeed);
    setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightSpeed);

    // Debug Output
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      // Monitor specifically Channel 1 and 3 as requested
      // Note: check your radio mapping. 
      // Usually Ch1 = Aileron/Steering, Ch3 = Throttle.
      int ch1_val = crsf.getChannel(1);
      int ch3_val = crsf.getChannel(3);
      
      Serial.print("Ch1: "); Serial.print(ch1_val);
      Serial.print(" Ch3: "); Serial.print(ch3_val);
      Serial.print(" | L: "); Serial.print(leftSpeed);
      Serial.print(" R: "); Serial.println(rightSpeed);
      lastPrint = millis();
    }

  } else {
    // Link Down / Failsafe
    stopMotors();
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      Serial.println("LINK DOWN - STOPPING");
      lastPrint = millis();
    }
  }
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

void stopMotors() {
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
}
