#ifndef SERVOS_H
#define SERVOS_H


#define N_SERVOS 1

// This delay in milliseconds determines determines the pause 
// between each one degree step the servo travels.  Increasing 
// this number will make the servo sweep more slowly.  
// Decreasing this number will make the servo sweep more quickly.  
#define SWEEP_COMMAND_INTERVAL 150 // ms

byte servoPins [N_SERVOS] = { 3 };
byte servoInitPosition [N_SERVOS] = { 90 }; // degrees


class SweepServo
{
  public:
    SweepServo();
    void initServo(int servoPin, int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif
