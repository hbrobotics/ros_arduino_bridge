/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/

#ifdef USE_SERVOS2

// NOTE: ServoTimer2.cpp moves the servo to 90 degrees after
// an attach() commmand so we set targetPositionDegrees to 90
// when constructing the servo object.

// Constructor
SweepServo2::SweepServo2()
{
  this->currentPositionDegrees = 90;
  this->targetPositionDegrees = 90; 
  this->lastSweepCommand = 0;
}

// Init
void SweepServo2::initServo(
    int servoPin,
    int stepDelayMs)
{
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = 90;
  this->targetPositionDegrees = 90;
  this->lastSweepCommand = millis();
  this->servo.attach(servoPin);
}

// Init
void SweepServo2::setServoDelay(
    int servoPin,
    int stepDelayMs)
{
  this->stepDelayMs = stepDelayMs;
}

// Perform Sweep
void SweepServo2::moveServo()
{

  // Get ellapsed time
  int delta = millis() - this->lastSweepCommand;

  // Check if time for a step
  if (delta > this->stepDelayMs) {
    // Check step direction
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;
      this->servo.write(this->currentPositionDegrees);
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;
      this->servo.write(this->currentPositionDegrees);
    }
    // if target == current position, do nothing

    // reset timer
    this->lastSweepCommand = millis();
  }
}

// Set a new target position
void SweepServo2::setTargetPosition(int position)
{
  this->targetPositionDegrees = position;
}

// Get the current servo position
int SweepServo2::getCurrentPosition()
{
  return this->currentPositionDegrees;
}

// Check whether we have already configured this servo
bool haveServo(int pin) {
  int i;
  for (i = 0; i < nServos; i++) {
    if (myServoPins[i] == pin) return true;
  }
  return false;
}

// Accessor for servo object
Servo SweepServo2::getServo()
{
  return this->servo;
}


#endif
