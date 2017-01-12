#define DEBUG 1
#include <PreparationH.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

RobotController* RC;

float blend = 0.1;

#define SAMPLES_PER_CYCLE 10
#define N_PERIODS 50
#define N_SAMPLES SAMPLES_PER_CYCLE * N_PERIODS
float samples[N_SAMPLES];

void initSamples() 
{
  float cyclesPerPeriod = 1.0/(float)SAMPLES_PER_CYCLE;
  for (int i = 0; i < N_SAMPLES; i++) {
    samples[i] = sin(2*PI*i*cyclesPerPeriod);
    samples[i] += sin(2*PI*i*1.1*cyclesPerPeriod);
    samples[i] += sin(2*PI*i*1.2*cyclesPerPeriod);
    samples[i] += sin(2*PI*i*1.3*cyclesPerPeriod);
    samples[i] += sin(2*PI*i*0.90*cyclesPerPeriod);
    samples[i] += sin(2*PI*i*0.80*cyclesPerPeriod);
    samples[i] += sin(2*PI*i*0.70*cyclesPerPeriod);
  }
}

void setup() {
  RC = new RobotController();
  RC->initialize();
  initSamples();
  Serial.begin(9600);
  delay(10000);
  _DC("Starting"); _NL;
  Serial.println("Really starting");
  for (int freq = 1000; freq < 10000; freq += 10) {
    int periodMicros = 1000000 / (SAMPLES_PER_CYCLE * freq);
    float average = 0.0;
    for (int i = 0; i < N_SAMPLES; i++) {
      average += analogRead(SOUND) * samples[i];
      delayMicroseconds(periodMicros);
      //_D(i); _D(freq); _D(periodMicros); _D(average); _NL;
    }
    _D(freq); _D(average); _NL;
  }
}


void loop() {
}
