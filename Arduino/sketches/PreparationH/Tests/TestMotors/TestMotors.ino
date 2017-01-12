#include <PreparationH.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

RobotController* RC;

void setup() {
  RC = new RobotController();
  RC->initialize();
  RC->go();
  delay(5000);
  RC->stop();
}

void loop() {
  // put your main code here, to run repeatedly:

}
