#ifndef PREPARATION_H
#define PREPARATION_H
/////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Sensor.h>

//#define DEBUG
//#undef DEBUG

#ifdef DEBUG
#define _D(X) Serial.print(#X " "), Serial.print(X), Serial.print("; ") 
#define _DC(X) Serial.print(#X "; ")
#define _NL Serial.print("\n")
#else
#define _D
#define _DC
#define _NL
#endif

// Digital I/O pin assignments
#define UNAVAILABLE 0
#define UNAVAILABLE 1
#define UNAVAILABLE 2
#define UNAVAILABLE 3
#define EXTINGUISHER 4
#define FRONT_SONAR 4
#define LEFT_FRONT_SONAR 5
#define RIGHT_FRONT_SONAR 6
#define LEFT_BACK_SONAR 7
#define RIGHT_BACK_SONAR 8
#define RIGHT_BACK_SONAR 11
#define ODOMETRY 9
#define LEFT_BUMPER 10
#define RIGHT_BUMPER 11
#define WHITE_LINE 12
#define LED 13

// Analog I/O pin assignments
#define SOUND 0
#define UV 1
#define FRONT_IR 2
#define UNASSIGNED 3
#define UNASSIGNED 4
#define UNASSIGNED 5

// Sensor conditions
#define WALL_FL 0x0001
#define NO_WALL_FL 0x0002
#define WALL_FR 0x0004
#define NO_WALL_FR 0x0008
#define WALL_BL 0x0010
#define NO_WALL_BL 0x0020
#define WALL_BR 0x0040
#define NO_WALL_BR 0x0080
#define WALL_FC 0x0100
#define NO_WALL_FC 0x0200
#define WALL_LEFT (WALL_FL|WALL_BL)
#define WALL_RIGHT (WALL_FR|WALL_BR)
#define OPEN_LEFT (NO_WALL_FL|NO_WALL_BL)
#define OPEN_RIGHT (NO_WALL_FR|NO_WALL_BR)

// I2C address assignments
#define MOTOR_CONTROLLER_ADDR 0x60
#define THERMAL_ARRAY_ADDR 0x68
#define ACCELEROMETER_ADDR xxxx

// Motor controller  assignments
#define RIGHT_MOTOR_ID 1
#define LEFT_MOTOR_ID 2
#define MAX_SPEED 1023
#define DEFAULT_SPEED (MAX_SPEED/2)
#define MOTOR_CORRECTION_WEIGHT 0.05

// Pan head servo settings
#define SERVO_FRONT 15
#define SERVO_LEFT 30
#define SERVO_RIGHT 1

// Devantech TPA81 thermal array constants 
#define REVISION 0x00
#define AMBIENT 0x01
#define PIXEL0TEMP 0x02
#define PIXEL1TEMP 0x02
#define PIXEL2TEMP 0x03
#define PIXEL3TEMP 0x04
#define PIXEL4TEMP 0x05
#define PIXEL5TEMP 0x06
#define PIXEL6TEMP 0x07
#define PIXEL7TEMP 0x08

#define ROBOT_RADIUS 5.0
#define WALL_NEAR_THRESH (ROBOT_RADIUS+2.0)
#define MAX_CORRECTION_DIFF 1.0
#define MAX_DIFF_TO_CORRECT 4
#define MAX_WALL_DISTANCE 24.0
#define MIN_COLLISION_DISTANCE 2.0
#define IR_COLLISION_DISTANCE 450

#define NUM_ODOMETER_TICKS 32
#define WHEEL_DIAMETER_INCHES 2.5
// This comes out to about 0.25 inches/tick.  At 1 ft/sec, this is about 20 ms/tick
#define INCHES_PER_TICK (3.14159*WHEEL_DIAMETER_INCHES/NUM_ODOMETER_TICKS)
#define TURN_90 20
#define TURN_180 40

#define FRONT_SONAR_OFFSET 1.57
#define BACK_SONAR_OFFSET 0.73

#define ABS(x) ((x)<0 ? -(x) : (x))

/*
class Effector
{
public:
  float curVal;
  float prevVal;
  unsigned long curTime;
  unsigned long prevTime;
}
*/

typedef void (*RunFunc)(int);

class RobotController
{
public:
  void initialize();
  void go(int rightSpeed, int leftSpeed, int rightDirection, int leftDirection); 
  void go(int speed=DEFAULT_SPEED, int direction=FORWARD); 
  void stop();
  bool readBumper(int bumper);
  bool readWhiteLine();
  float readOdometry();
  float readVirtualOdometry();
  bool updateOdometry();
  int readUv();
  float readDistanceInfrared();
  float readDistanceSonar(int sensor);
  void setPanAngle(float angle);
  void readThermalArray(unsigned char* pixels);
  void follow(int speed=DEFAULT_SPEED);
  void updateSensors();
  void avoid(int speed=DEFAULT_SPEED);
  void run(RunFunc func, int speed, int exitConditions, float minDist = 0, float maxDist = 10000);
  void turnLeft(int count, int speed = DEFAULT_SPEED);
  void turnRight(int count, int speed = DEFAULT_SPEED);
  float getSpeed();

  Sensor sonarFR;
  Sensor sonarFL;
  Sensor sonarBR;
  Sensor sonarBL;
  Sensor sonarFC;
  Sensor infrared;
  Sensor uv;
  Sensor odometer;
  Sensor virtualOdometer;
  Sensor whiteline;
  Sensor bumperFR;
  Sensor bumperFL;

private:
  Adafruit_MotorShield motorController;
  Adafruit_DCMotor* rightMotor;
  Adafruit_DCMotor* leftMotor;
  int prevTickTime;
  int avgTimePerTick;
  int curLeftMotorSpeed;
  int curRightMotorSpeed;
  int curLeftMotorDirection;
  int curRightMotorDirection;
  float curPanAngle;
  bool curOdometerWheel;
  /*
  Effector pan;
  Effector motorR;
  Effector motorL;
  */
};


/////////////////////////////////////////////////////////////////////////////
#endif
