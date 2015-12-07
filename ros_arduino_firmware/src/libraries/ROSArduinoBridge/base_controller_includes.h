/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE

  /* The Pololu VNH5019 dual motor driver shield */
  #define POLOLU_VNH5019
  
  /* The Pololu MC33926 dual motor driver shield */
  //#define POLOLU_MC33926

  /* The RoboGaia encoder shield */
  #define ROBOGAIA
  
  #ifdef ROBOGAIA
    #include "MegaEncoderCounter.h"
  #endif
  
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Encoders directly attached to Arduino board */
  #undef ARDUINO_ENC_COUNTER

#endif

