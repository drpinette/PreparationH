#include <PreparationH.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

RobotController* RC;

void follow(int speed) { RC->follow(MAX_SPEED); }

void setup() {
  RC = new RobotController();
  RC->initialize();
  delay(3000);
  RC->run(follow, MAX_SPEED, WALL_FC);
  RC->stop();
  RC->turnLeft(TURN_90);
  RC->run(follow, MAX_SPEED, NO_WALL_BR);
  RC->stop();
  RC->turnLeft(TURN_180);
  RC->run(follow, MAX_SPEED, WALL_FC);
  RC->stop();
  RC->turnRight(TURN_90);
  RC->run(follow, MAX_SPEED, NO_WALL_BR);
  RC->stop();
}

void loop() {
}
