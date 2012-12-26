/***************************************************************
   Motor driver definitions
   
   Add a #ifdef block to this file to include support for
   a particular motor driver.  Then add the appropriate #define
   near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
   
#ifdef POLOLU_VNH5019
  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#endif


