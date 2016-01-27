#ifndef SERVOS2_H
#define SERVOS2_H

#define MAX_N_SERVOS 48

// This delay in milliseconds determines the pause 
// between each one degree step the servo travels.  Increasing 
// this number will make the servo sweep more slowly.  
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.

class SweepServo2
{
  public:
    SweepServo2();
    
    void initServo(
        int servoPin,
        unsigned long stepDelayMs);

     void setServoDelay(
        int servoPin,
        unsigned long stepDelayMs);
        
    void moveServo();
    void setTargetPosition(int position);
    void setServoDelay(int delay);
    int getCurrentPosition();
    void enable();
    void disable();
    bool isEnabled();
    
    Servo getServo();

  private:
    Servo servo;
    unsigned long stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    unsigned long lastSweepCommand;
    bool enabled;
};

SweepServo2 myServos [MAX_N_SERVOS];

int myServoPins [MAX_N_SERVOS];

#endif

