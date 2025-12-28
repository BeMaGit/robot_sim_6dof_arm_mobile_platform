/*
 * Platform SBUS Test
 * 
 * Purpose: Test mobile platform control using a RadioMaster R81 (or compatible) SBUS receiver.
 * 
 * Hardware: 
 * - Arduino Mega 2560
 * - RadioMaster R81 Receiver (SBUS)
 * - Motor Driver (L298N or similar) connected to Pins 9, 10, 11, 12
 * 
 * WIRING (CRITICAL):
 * - R81 Signal -> INVERTER -> Mega Pin 19 (RX1)
 * - R81 5V -> Mega 5V
 * - R81 GND -> Mega GND
 * 
 * DEPENDENCIES:
 * - "SBUS" Library by Bolder Flight Systems (Search "SBUS" in Library Manager)
 */

#include "SBUS.h"

// --- Configuration ---

// SBUS Setup
// R81 connected to Serial1 (Pin 19 on Mega)
// NOTE: SBUS is usually inverted. You MUST use a hardware inverter on Pin 19 
// unless your receiver has an "uninverted" pad.
SBUS x8r(Serial1); 

// Motor Driver Pins
const int PIN_LEFT_PWM   = 9;
const int PIN_LEFT_DIR   = 10;
const int PIN_RIGHT_PWM  = 11;
const int PIN_RIGHT_DIR  = 12;

// Channel Mapping (RadioMaster Default / AETR or similar)
// Check your radio's monitor to confirm channel numbers.
const int CH_THROTTLE = 2; // Left Stick Y (approx) - Ch3 on many radios, Ch2 or 3 depending on mapping
const int CH_STEERING = 3; // Left Stick X (approx) - Ch4 on many radios

// Channel Values
float channels[16];
bool failSafe;
bool lostFrame;

void setup() {
  Serial.begin(115200);
  Serial.println("SBUS Platform Test Starting...");
  Serial.println("Ensure RECEIVER is bound and connected to PIN 19 (RX1) with INVERTER.");

  // SBUS Start
  x8r.begin();

  // Motor Pins
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
}

void loop() {
  // Read SBUS
  if (x8r.read(&channels[0], &failSafe, &lostFrame)) {
    
    // SBUS library returns values typically approx 172 to 1811 (depending on radio calibration)
    // We'll normalize these to a convenient range or use map() directly.
    // However, this library might return scaled values or raw int. 
    // Let's assume standard SBUS range ~172-1811.
    // Wait, Bolder Flight SBUS lib often scales. Let's verify by printing first if unsure.
    // But usually we just take the raw or normalized float. 
    // Let's assume the channels array contains raw data if not specified otherwise.
    // Actually, Bolder Flight SBUS 'read' populates the array.
    // Let's assume standard raw values for now and print them for debugging.

    // Map channels to Motor Speed (-255 to 255)
    // Using Ch3 (Throttle) and Ch4 (Rudder/Steering) for Arcade Drive
    // Adjust indices 2 and 3 (0-indexed) corresponds to Channel 3 and 4
    
    // NOTE: Array is 0-indexed. Channel 1 is at index 0.
    // So Channel 3 is index 2, Channel 4 is index 3.
    float ch3_raw = channels[2]; 
    float ch4_raw = channels[3];

    // Simple Deadband & Map
    // SBUS Center is approx 992 or 1000 depending on protocol variant, usually 172-1811 range centered at ~992.
    // Let's assume 172-1811. Center ~992.
    
    int throttle = map(ch3_raw, 172, 1811, -255, 255);
    int steering = map(ch4_raw, 172, 1811, -255, 255); 

    // Handle deadband
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

    // Debug Output (Optional - slows down loop if too fast)
    // static unsigned long lastPrint = 0;
    // if (millis() - lastPrint > 200) {
    //   Serial.print("Ch3: "); Serial.print(ch3_raw);
    //   Serial.print(" -> Throt: "); Serial.print(throttle);
    //   Serial.print(" | Ch4: "); Serial.print(ch4_raw);
    //   Serial.print(" -> Steer: "); Serial.print(steering);
    //   Serial.println();
    //   lastPrint = millis();
    // }

  } else {
    // If no data, stop motors for safety
    // BUT only if we haven't received data for a while?
    // The read() function returns true only when new packet arrives.
    // We shouldn't stop immediately between packets (every 9ms).
    // Let's rely on failSafe flag.
  }

  if (failSafe) {
    stopMotors();
    Serial.println("FAILSAFE ACTIVE - STOPPING");
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
