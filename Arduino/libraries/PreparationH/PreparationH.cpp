#include <PreparationH.h>

void RobotController::initialize()
{
  pinMode(ODOMETRY, INPUT_PULLUP);
  pinMode(WHITE_LINE, INPUT_PULLUP);
  pinMode(RIGHT_BUMPER, INPUT_PULLUP);
  pinMode(LEFT_BUMPER, INPUT_PULLUP);

  motorController = Adafruit_MotorShield(MOTOR_CONTROLLER_ADDR);
  rightMotor = motorController.getMotor(RIGHT_MOTOR_ID);
  leftMotor = motorController.getMotor(LEFT_MOTOR_ID);
  motorController.begin();

  Wire.begin();  // Set up Arduino as I2C master
  setPanAngle(0);  // Make the pan head face front

  curLeftMotorSpeed = 0;
  curRightMotorSpeed = 0;
  curLeftMotorDirection = RELEASE;
  curRightMotorDirection = RELEASE;

  curOdometerWheel = digitalRead(ODOMETRY);
  avgTimePerTick = 0;
  prevTickTime = 0;
}

void RobotController::go(int rightSpeed, int leftSpeed, int rightDirection, int leftDirection)
{
  if (rightDirection != curRightMotorDirection) rightMotor->run(RELEASE);
  if (leftDirection != curLeftMotorDirection) leftMotor->run(RELEASE);
  if (rightSpeed != curRightMotorSpeed) rightMotor->setSpeed(rightSpeed);
  if (rightDirection != curRightMotorDirection) rightMotor->run(rightDirection);
  if (leftSpeed != curLeftMotorSpeed) leftMotor->setSpeed(leftSpeed);    
  if (leftDirection != curLeftMotorDirection) leftMotor->run(leftDirection);
  curRightMotorSpeed = rightSpeed;
  curRightMotorDirection = rightDirection;
  curLeftMotorSpeed = leftSpeed;
  curLeftMotorDirection = leftDirection;
}

void RobotController::go(int speed, int direction)
{
  go(speed, speed, direction, direction);
}

// Return estimated total distance traveled in inches
float RobotController::readVirtualOdometry()
{
  float distance = virtualOdometer.curTime * getSpeed();
  virtualOdometer.update(distance);
  return distance * INCHES_PER_TICK;
}

void RobotController::stop()
{
  go(0, BRAKE);
  delay(200); // Braking delay to give time to stop
}

bool RobotController::readBumper(int bumper)
{
  // Closed switch pulls voltage to 0, hence invert
  return !digitalRead(bumper);
}

bool RobotController::readWhiteLine()
{
  // White reflection pulls transistor to 0, hence invert
  return !digitalRead(WHITE_LINE);
}

bool RobotController::updateOdometry()
{
  bool val = digitalRead(ODOMETRY);
  if (val != curOdometerWheel) {
      odometer.curVal++;
      curOdometerWheel = val;
      unsigned long curTickTime = millis();
      if (prevTickTime != 0) {
	const float k = 0.9;
	avgTimePerTick = avgTimePerTick*k + (1-k)*(curTickTime - prevTickTime);
      }
      prevTickTime = curTickTime;
  }
  return val;
}

// Get speed in ticks/ms
float RobotController::getSpeed()
{
  return 1.0/(float)avgTimePerTick;
}

// Return measured total distance traveled in inches
float RobotController::readOdometry()
{
  float distance = odometer.curVal;
  odometer.update(distance);
  return distance * INCHES_PER_TICK;
}

int RobotController::readUv()
{
  int val = analogRead(UV);
  uv.update(val);
  return val;
}

float RobotController::readDistanceInfrared()
{
  // TODO: Calibrate this to inches
  int distance = analogRead(FRONT_IR);;
  infrared.update(distance);
  return distance;
}

float RobotController::readDistanceSonar(int sensor)
{
  // Make sure sensor is valid
  Sensor* sonar;
  float offset;
  switch (sensor) {
  case FRONT_SONAR:
    sonar = &sonarFC;
    offset = 0;
    break;
  case LEFT_FRONT_SONAR:
    sonar = &sonarFL;
    offset = FRONT_SONAR_OFFSET;
    break;
  case RIGHT_FRONT_SONAR:
    sonar = &sonarFR;
    offset = FRONT_SONAR_OFFSET;
    break;
  case LEFT_BACK_SONAR:
    sonar = &sonarBL;
    offset = BACK_SONAR_OFFSET;
    break;
  case RIGHT_BACK_SONAR:
    sonar = &sonarBR;
    offset = BACK_SONAR_OFFSET;
    break;
  default:
    return 0;
  }
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(sensor, OUTPUT);
  digitalWrite(sensor, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor, HIGH);
  delayMicroseconds(5);
  digitalWrite(sensor, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(sensor, INPUT);
  int duration = pulseIn(sensor, HIGH);

  // Convert to inches
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  float distance = ((float)duration) / 74.0 / 2.0;
  distance +=  offset;
  // A working sonar should never return a tiny value, since it can't measure that small. A safer value to return is the previous value of the sonar.
  if (distance < 0.01)  return sonar->curVal;
  else {
      sonar->update(distance);
      return distance;
  }
}

void RobotController::setPanAngle(float angle)
{
  // Calculate the servo setting from the angle, -90 is left, +90 is right, 0 is forward
  if (angle < -90.0) curPanAngle = -90.0;
  else if (angle > 90.0) curPanAngle = 90.0;
  else curPanAngle = angle;
  int servoSetting = SERVO_FRONT;
  if (curPanAngle > 0) servoSetting = (int)(((90.0-angle)/90.0)*(SERVO_FRONT-SERVO_RIGHT)+0.5)+SERVO_RIGHT;
  else if (curPanAngle < 0) servoSetting = (int)((-angle/90.0)*(SERVO_LEFT-SERVO_FRONT)+0.5)+SERVO_FRONT;
  // Set the servo via the termal sensor array
  Wire.beginTransmission(THERMAL_ARRAY_ADDR);
  Wire.write(REVISION);    
  Wire.write((unsigned char)servoSetting);
  Wire.endTransmission();
}

void RobotController::readThermalArray(unsigned char* pixels)
{
  // 0-th element of array is ambient, rest are pixels 0-7
  unsigned char* curPix = pixels;
  for (int i = 0; i < 9; i++, curPix++) {
    Wire.beginTransmission(THERMAL_ARRAY_ADDR);
    Wire.write(AMBIENT+i);    
    Wire.endTransmission();
    Wire.requestFrom(THERMAL_ARRAY_ADDR, 1);
    while(!Wire.available());
    *curPix = Wire.read();    
  }
}

void RobotController::follow(int speed)
{
  // Use only distances that are from a nearby wall
  int useRightBackDist = sonarBR.curVal < MAX_WALL_DISTANCE ? 1 : 0;
  int useRightFrontDist = sonarFR.curVal < MAX_WALL_DISTANCE ? 1 : 0;
  int useLeftBackDist = sonarBL.curVal < MAX_WALL_DISTANCE ? 1 : 0;
  int useLeftFrontDist = sonarFL.curVal < MAX_WALL_DISTANCE ? 1 : 0;

  // Calcuate distance of robot from each way
  float rightDist = 0.0;
  if (useRightFrontDist + useRightBackDist > 0) rightDist = (useRightFrontDist*sonarFR.curVal + useRightBackDist*sonarBR.curVal)/(useRightFrontDist+useRightBackDist);
  float leftDist = 0.0;
  if (useLeftFrontDist + useLeftBackDist > 0) leftDist = (useLeftFrontDist*sonarFL.curVal + useLeftFrontDist*sonarBL.curVal)/(useLeftFrontDist+useLeftBackDist);

  // Do we need to angle the robot away from the wall to maintain clearance?
  float correctionDiff = 0;
  if (rightDist < WALL_NEAR_THRESH) {
    if (rightDist < ROBOT_RADIUS) correctionDiff = MAX_CORRECTION_DIFF;
    else correctionDiff = MAX_CORRECTION_DIFF * (1.0 - (rightDist - ROBOT_RADIUS) / (WALL_NEAR_THRESH - ROBOT_RADIUS));
  } else if (leftDist < WALL_NEAR_THRESH) {
    if (leftDist < ROBOT_RADIUS) correctionDiff = -MAX_CORRECTION_DIFF;
    else correctionDiff = -MAX_CORRECTION_DIFF * (1.0 - (leftDist - ROBOT_RADIUS) / (WALL_NEAR_THRESH - ROBOT_RADIUS));
  } 
  // Compute the degree of correction needed to maintain the correction difference
  float rightDiff = sonarFR.curVal - sonarBR.curVal;
  float leftDiff = sonarBL.curVal - sonarFL.curVal;
  bool useRightDiff =  ABS(rightDiff) < MAX_DIFF_TO_CORRECT && rightDist < MAX_WALL_DISTANCE;
  bool useLeftDiff = ABS(leftDiff) < MAX_DIFF_TO_CORRECT && leftDist < MAX_WALL_DISTANCE;
  float totalWeight = 0;
  float totalCorrection = 0;
  //useLeftDiff = false;

  // Add in correction from differences of front and back sonar sensor pairs
  if (useRightDiff) {
    totalCorrection += correctionDiff - rightDiff;
    totalWeight += 1.0;
  }
  if (useLeftDiff) {
    totalCorrection +=  correctionDiff - leftDiff;
    totalWeight += 1.0;
  }

  // Add in correction for prev and cur value for sonar sensor pairs.  Less accurate, so it gets less weight
  const float singleSensorWeight  = 0.1;
  float distanceTraveled = getSpeed() * (virtualOdometer.curVal - virtualOdometer.prevVal);
  float sensorDiffScaleFactor = 2 * ROBOT_RADIUS / distanceTraveled;
  if (useRightBackDist && sonarBR.prevVal < MAX_WALL_DISTANCE) {
    float diff = sonarBR.curVal - sonarBR.prevVal;
    totalCorrection += singleSensorWeight * diff;
    totalWeight += singleSensorWeight;
  }
  if (useRightFrontDist && sonarFR.prevVal < MAX_WALL_DISTANCE) {
    float diff = sonarFR.curVal - sonarFR.prevVal;
    totalCorrection += singleSensorWeight * diff;
    totalWeight += singleSensorWeight;
  }
  if (useLeftBackDist && sonarBL.prevVal < MAX_WALL_DISTANCE) {
    float diff = sonarBL.curVal - sonarBL.prevVal;
    totalCorrection += singleSensorWeight * diff;
    totalWeight += singleSensorWeight;
  }
  if (useLeftFrontDist && sonarFL.prevVal < MAX_WALL_DISTANCE) {
    float diff = sonarFL.curVal - sonarFL.prevVal;
    totalCorrection += singleSensorWeight * diff;
    totalWeight += singleSensorWeight;
  }

  //Serial.print("totalCorrection: "); Serial.println(totalCorrection);
  if (totalWeight > 0) totalCorrection /= (totalWeight * MAX_CORRECTION_DIFF);
  if (totalCorrection > 1.0) totalCorrection = 1.0;
  else if (totalCorrection < -1.0) totalCorrection = -1.0;

  // Turn the correction into motor correction
  int leftSpeed = speed;
  int rightSpeed = speed;
  if (totalCorrection > 0) leftSpeed -= (int)(MOTOR_CORRECTION_WEIGHT*speed*totalCorrection);
  else if (totalCorrection < 0) rightSpeed += (int)(MOTOR_CORRECTION_WEIGHT*speed*totalCorrection);
  go(rightSpeed, leftSpeed, FORWARD, FORWARD);

  /*
  Serial.print("R: "); Serial.print(rightDiff); Serial.print(", "); Serial.print(rightDist); Serial.print(", "); Serial.println(rightSpeed);
  Serial.print("L: "); Serial.print(leftDiff); Serial.print(", "); Serial.print(leftDist); Serial.print(", "); Serial.println(leftSpeed);
  Serial.print("correctionDiff: "); Serial.print(correctionDiff); Serial.print(", totalCorrection: "); Serial.println(totalCorrection);
  */
}

void RobotController::updateSensors()
{
  readDistanceSonar(LEFT_FRONT_SONAR);
  updateOdometry();
  readDistanceSonar(RIGHT_FRONT_SONAR);
  updateOdometry();
  readDistanceSonar(LEFT_BACK_SONAR);
  updateOdometry();
  readDistanceSonar(RIGHT_BACK_SONAR);
  updateOdometry();
  readDistanceInfrared();
  //readDistanceSonar(FRONT_SONAR);
  //updateOdometry();
 }

void RobotController::avoid(int speed)
{
  float dist;
  // Initiate a turn, if needed
  if (sonarFR.curVal < sonarFL.curVal && sonarFR.curVal < ROBOT_RADIUS) {
    dist = sonarFR.curVal;
    //go(speed, speed, BRAKE, BACKWARD);
    go(0, 0, BRAKE, BRAKE);
    //go(speed, speed, BACKWARD, BACKWARD);
  } else if (sonarFL.curVal < sonarFR.curVal && sonarFL.curVal < ROBOT_RADIUS) {
    dist = sonarFL.curVal;
    //go(speed, speed, BACKWARD, BRAKE);
    go(0, 0, BRAKE, BRAKE);
    //go(speed, speed, BACKWARD, BACKWARD);
  } else return;
  // Wait for turn to complete
  int maxTime = 500;
  int waitTime = (ROBOT_RADIUS - dist)/ROBOT_RADIUS * maxTime;
  delay(waitTime);
  // Restore forward motion
  //go(speed, speed, FORWARD, FORWARD);
}

void RobotController::run(RunFunc func, int speed, int exitConditions, float minDist, float maxDist)
{
  float curOdometry = readOdometry();
  float minOdometry = curOdometry + minDist;
  float maxOdometry = curOdometry + maxDist;
  _D(curOdometry); _D(minOdometry); _D(maxOdometry); _NL;
  while (curOdometry < maxOdometry) {
    updateSensors();
    curOdometry = readOdometry();
    _D(curOdometry); _D(minOdometry); _D(maxOdometry); _NL;
    if (curOdometry > minOdometry) {
      bool done = true;
      bool hit = false;
      if (exitConditions & (WALL_FL|WALL_LEFT)) { 
	done = done && sonarFL.curVal <= MAX_WALL_DISTANCE;
        hit = hit || sonarFL.curVal <= MAX_WALL_DISTANCE;
	_D(sonarFL.curVal); _NL; 
      }
      if (exitConditions & (NO_WALL_FL|OPEN_LEFT)) { 
	done = done && sonarFL.curVal > MAX_WALL_DISTANCE; 
	hit = hit || sonarFL.curVal > MAX_WALL_DISTANCE; 
	_D(sonarFL.curVal); _NL; 
      }
      if (exitConditions & (WALL_BL|WALL_LEFT)) { 
	done = done && sonarBL.curVal <= MAX_WALL_DISTANCE; 
	hit = hit || sonarBL.curVal <= MAX_WALL_DISTANCE; 
	_D(sonarBL.curVal); _NL; 
      } 
      if (exitConditions & (NO_WALL_BL|OPEN_LEFT)) { 
	done = done && sonarBL.curVal > MAX_WALL_DISTANCE; 
	hit = hit || sonarBL.curVal > MAX_WALL_DISTANCE; 
	_D(sonarBL.curVal); _NL; 
      }
      if (exitConditions & (WALL_FR|WALL_RIGHT)) { 
	done = done && sonarFR.curVal <= MAX_WALL_DISTANCE; 
	hit = hit || sonarFR.curVal <= MAX_WALL_DISTANCE; 
	_D(sonarFR.curVal); _NL; 
      }
      if (exitConditions & (NO_WALL_FR|OPEN_RIGHT)) { 
	done = done && sonarFR.curVal > MAX_WALL_DISTANCE; 
	hit = hit || sonarFR.curVal > MAX_WALL_DISTANCE; 
	_D(sonarFR.curVal); _NL; 
      }
      if (exitConditions & (WALL_BR|WALL_RIGHT)) { 
	done = done && sonarBR.curVal <= MAX_WALL_DISTANCE; 
	hit = hit || sonarBR.curVal <= MAX_WALL_DISTANCE; 
	_D(sonarBR.curVal); _NL; 
      }
      if (exitConditions & (NO_WALL_BR|OPEN_RIGHT)) { 
	done = done && sonarBR.curVal > MAX_WALL_DISTANCE; 
	_D(sonarBR.curVal); _NL; 
      }
      if (exitConditions & WALL_FC) { 
	//done = done && sonarFC.curVal <= MIN_COLLISION_DISTANCE; 
	//hit = hit || sonarFC.curVal <= MIN_COLLISION_DISTANCE; 
	//_D(sonarFC.curVal); _NL;
	done = done && infrared.curVal > IR_COLLISION_DISTANCE; 
	hit = hit || infrared.curVal > IR_COLLISION_DISTANCE; 
	_D(infrared.curVal); _NL; 
      }
      if (exitConditions & NO_WALL_FC) {
	//done = done && sonarFC.curVal > MIN_COLLISION_DISTANCE; 
	//hit = hit || sonarFC.curVal > MIN_COLLISION_DISTANCE; 
	//_D(sonarFC.curVal); _NL; 
	done = done && infrared.curVal < IR_COLLISION_DISTANCE; 
	hit = hit || infrared.curVal < IR_COLLISION_DISTANCE; 
	_D(infrared.curVal); _NL; 
      }
      if (hit && done) return;
    }
    _DC ("Calling func"); _NL; _NL;
    func(speed);
  }
}

void RobotController::turnLeft(int count, int speed)
{
  int endVal = odometer.curVal + count;
  go(speed, speed, FORWARD, BACKWARD);
  while (odometer.curVal <= endVal) updateOdometry();
  stop();
}

void RobotController::turnRight(int count, int speed)
{
  int endVal = odometer.curVal + count;
  go(speed, speed, BACKWARD, FORWARD);
  while (odometer.curVal <= endVal) updateOdometry();
  stop();
}
