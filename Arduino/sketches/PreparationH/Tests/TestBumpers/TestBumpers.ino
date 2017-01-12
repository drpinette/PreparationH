#include <PreparationH.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

RobotController* RC;

void setup() {
  RC = new RobotController();
  RC->initialize();
}

void loop() {
    if (digitalRead(RIGHT_BUMPER)) digitalWrite(LED, LOW);
    else digitalWrite(LED, HIGH);
}
