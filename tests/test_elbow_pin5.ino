#include <Servo.h>

const int PIN_ELBOW = 5;
Servo testServo;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Elbow Servo Test on Pin 5");
  testServo.attach(PIN_ELBOW);
}

void loop() {
  Serial.println("Moving to 0...");
  testServo.write(0);
  delay(2000);
  
  Serial.println("Moving to 90...");
  testServo.write(90);
  delay(2000);
  
  Serial.println("Moving to 180...");
  testServo.write(180);
  delay(2000);
}
