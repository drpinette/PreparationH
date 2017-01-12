#include <Sensor.h>
#include <Arduino.h>

Sensor::Sensor() : curVal(0), prevVal(0), curTime(0), prevTime(0) 
{
}

float Sensor::update(float value)
{
  prevTime = curTime;
  curTime = millis();
  prevVal = curVal;
  curVal = value;
}
