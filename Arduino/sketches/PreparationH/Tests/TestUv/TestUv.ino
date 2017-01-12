#include <PreparationH.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

RobotController* RC;

void setup() {
  RC = new RobotController();
  RC->initialize();
}

int maxDelay = 0;

void loop() {
    int val = RC->readUv();
    if (val > maxDelay) maxDelay = val;
    if (val > 0) {
      digitalWrite(LED, HIGH);
      delay(val/10);
      digitalWrite(LED, LOW);
    }
    delay((maxDelay-val)/10);
}

