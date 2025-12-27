
#include <ArduinoJson.h>
#include <ServoEasing.hpp>

// --- Pin Definitions (Same as main sketch) ---
const int PIN_WAIST      = 2;
const int PIN_SHOULDER_1 = 3;
const int PIN_SHOULDER_2 = 4;
const int PIN_ELBOW      = 5;
const int PIN_WRIST_P    = 6;
const int PIN_WRIST_R    = 7;
const int PIN_GRIPPER    = 8;

const int PIN_LEFT_PWM   = 9;
const int PIN_LEFT_DIR   = 10;
const int PIN_RIGHT_PWM  = 11;
const int PIN_RIGHT_DIR  = 12;

ServoEasing sWaist, sShoulder1, sShoulder2, sElbow, sWristP, sWristR, sGripper;

void setup() {
  Serial.begin(115200);
  while (!Serial) continue;

  Serial.println("JSON Control Test Started");
  
  if (sWaist.attach(PIN_WAIST, 90) == INVALID_SERVO) Serial.println(F("Error attaching Waist"));
  if (sShoulder1.attach(PIN_SHOULDER_1, 90) == INVALID_SERVO) Serial.println(F("Error attaching Shoulder 1"));
  if (sShoulder2.attach(PIN_SHOULDER_2, 90) == INVALID_SERVO) Serial.println(F("Error attaching Shoulder 2"));
  if (sElbow.attach(PIN_ELBOW, 90) == INVALID_SERVO) Serial.println(F("Error attaching Elbow"));
  if (sWristP.attach(PIN_WRIST_P, 90) == INVALID_SERVO) Serial.println(F("Error attaching Wrist P"));
  if (sWristR.attach(PIN_WRIST_R, 90) == INVALID_SERVO) Serial.println(F("Error attaching Wrist R"));
  if (sGripper.attach(PIN_GRIPPER, 0) == INVALID_SERVO) Serial.println(F("Error attaching Gripper"));

  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    // Deserialize the JSON document
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, Serial);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    // --- Process Servos ---
    if (doc.containsKey("servos")) {
      JsonObject servos = doc["servos"];
      
      int speed = servos["speed"] | 60;
      setSpeedForAllServos(speed);

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
      
      Serial.println("Servos updated");
    }

    // --- Process Motors ---
    if (doc.containsKey("motors")) {
      JsonObject motors = doc["motors"];
      int left = motors["left"] | 0;
      int right = motors["right"] | 0;
      int duration = motors["duration"] | 0;

      setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, left);
      setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, right);
      
      Serial.print("Motors: L="); Serial.print(left); Serial.print(" R="); Serial.println(right);

      if (duration > 0) {
        delay(duration);
        setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, 0);
        setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, 0);
        Serial.println("Motors stopped after duration");
      }
    }
  }
}

void setSpeedForAllServos(int speed) {
  sWaist.setSpeed(speed);
  sShoulder1.setSpeed(speed);
  sShoulder2.setSpeed(speed);
  sElbow.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
  sGripper.setSpeed(speed*2);
}

void setMotor(int pinPWM, int pinDir, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinDir, LOW);
    analogWrite(pinPWM, abs(speed));
  }
}
