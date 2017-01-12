#include <PreparationH.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

RobotController* RC;

int i = 0;

void setup() {
  RC = new RobotController();
  pinMode(12,INPUT);
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  //if (digitalRead(12) == HIGH) digitalWrite(13,HIGH);
  //else digitalWrite(13,LOW);
  int val = analogRead(3);
  Serial.print(i); Serial.print(" ");
  Serial.println(val);
  delay(1000);
  i++;
}
