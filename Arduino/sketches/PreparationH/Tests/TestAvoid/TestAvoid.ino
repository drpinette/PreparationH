vinclude <PreparationH.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

RobotController* RC;

void setup() {
  RC = new RobotController();
  RC->initialize();
}

void loop() {
  RC->updateSensors();
  RC-avoid(MAX_SPEED);
  RC->follow(MAX_SPEED);
  //delay(5000);
}
