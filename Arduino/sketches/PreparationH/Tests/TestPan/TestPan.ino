
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
    if (RC->readWhiteLine()) digitalWrite(LED, HIGH);
    else digitalWrite(LED, LOW);
}

