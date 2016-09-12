/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen, Nathaniel Gallinger

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//#define USE_BASE      // Enable/disable the base controller code

//#define USE_IMU       // Enable/disable use of an IMU

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
  /* The Pololu VNH5019 dual motor driver shield */
  #define POLOLU_VNH5019

  /* The Pololu MC33926 dual motor driver shield */
  //#define POLOLU_MC33926

  /* The Adafruit motor shield V2 */
  //#define ADAFRUIT_MOTOR_SHIELD_V2

  /* The Ardunino Motor Shield R3 */
  //#define ARDUINO_MOTOR_SHIELD_R3
  
  /* The brake uses digital pins 8 and 9 and is not compatible with the Robogaia 3-axis
  *  endcoder shield.  Cut the brake jumpers on the R3 motor shield if you want to use
  *  it with the 3-axis encoder shield.
  */
  //#define USE_ARDUINO_MOTOR_SHIELD_R3_BRAKE
  
  /* For testing only */
  // #define NO_MOTOR_CONTROLLER
  
  /* The RoboGaia encoder shield */
  #define ROBOGAIA
  
  /* The RoboGaia 3-axis encoder shield */
  //#define ROBOGAIA_3_AXIS
  
  /* Encoders directly attached to Arduino board */
  //#define ARDUINO_ENC_COUNTER
#endif

//#define USE_SERVOS  // Enable/disable use of old PWM servo support as defined in servos.h

#define USE_SERVOS2  // Enable/disable use of new PWM servo support as defined in servos2.h

/* Include old servo support if required */
#ifdef USE_SERVOS
   #include "Servo.h"
   #include "servos.h"

/* Include new servo support if desired */
#elif defined(USE_SERVOS2)
   #include "Servo.h"
   #include "servos2.h"
   int nServos = 0;
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  #ifdef ROBOGAIA_3_AXIS
    // The sensor communicates using SPI, so include the library:
    #include <SPI.h>
  #endif

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

#ifdef USE_IMU
  #include "imu.h"

  // The only IMU currently supported is the Adafruit 9-DOF IMU
  #define ADAFRUIT_9DOF

#endif

/* Variable initialization */
#define BAUDRATE     57600

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[32];
char argv2[32];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  String output;

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.print(F(" "));
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println(F("OK")); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println(F("OK")); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println(F("OK"));
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_IMU
  case READ_IMU:
    imu_data = readIMU();
    /* Send the IMU data base in the following order
     * [ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, ch]
     */
    Serial.print(imu_data.ax);
    Serial.print(F(" "));
    Serial.print(imu_data.ay);
    Serial.print(F(" "));
    Serial.print(imu_data.az);
    Serial.print(F(" "));
    Serial.print(imu_data.gx);
    Serial.print(F(" "));
    Serial.print(imu_data.gy);
    Serial.print(F(" "));
    Serial.print(imu_data.gz);
    Serial.print(F(" "));
    Serial.print(imu_data.mx);
    Serial.print(F(" "));
    Serial.print(imu_data.my);
    Serial.print(F(" "));
    Serial.print(imu_data.mz);
    Serial.print(F(" "));
    Serial.print(imu_data.roll);
    Serial.print(F(" "));
    Serial.print(imu_data.pitch);
    Serial.print(F(" "));
    Serial.println(imu_data.ch);
    break;
#endif
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println(F("OK"));
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#elif defined(USE_SERVOS2)
  case CONFIG_SERVO:
    if (!haveServo(arg1)) {
      myServos[arg1].initServo(arg1, arg2);
      myServoPins[nServos] = arg1;
      myServos[arg1].enable();
      nServos++;
    }
    Serial.println(F("OK"));
    break;
  case SERVO_WRITE:
    if (myServos[arg1].isEnabled()) {
      myServos[arg1].setTargetPosition(arg2);
    }
    Serial.println(F("OK"));
    break;
  case SERVO_READ:
    Serial.println(myServos[arg1].getCurrentPosition());
    break;
  case SERVO_DELAY:
    myServos[arg1].setServoDelay(arg1, arg2);
    Serial.println(F("OK"));
    break;
  case DETACH_SERVO:
    myServos[arg1].getServo().detach();
    myServos[arg1].disable();
    Serial.println(F("OK"));
    break;
  case ATTACH_SERVO:
    if (!haveServo(arg1)) {
      myServos[arg1].initServo(arg1, 0);
      myServoPins[nServos] = arg1;
      nServos++;
    }
    else {
      myServos[arg1].getServo().attach(arg1);  
    }
    myServos[arg1].enable();
    Serial.println(F("OK"));
    break;

#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(F(" "));
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println(F("OK"));
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println(F("OK")); 
    break;
  case UPDATE_PID:
    i = 0;
    while ((str = strtok_r(p, ":", &p)) != '\0') {
      pid_args[i] = atoi(str);
      i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println(F("OK"));
    break;
#endif
  default:
    Serial.println(F("Invalid Command"));
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  // Initialize the motor controller if used */
  #ifdef USE_BASE
    /* Initialize the encoder interface */
    initEncoders();
    initMotorController();
    resetPID();
  #endif

  #ifdef USE_IMU
    initIMU();
  #endif

  /* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  // If we are using base control, run a PID calculation at the appropriate intervals
  #ifdef USE_BASE
    if (millis() > nextPID) {
      updatePID();
      nextPID += PID_INTERVAL;
    }
  
    // Check to see if we have exceeded the auto-stop interval
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
  #endif

  // Sweep servos
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].doSweep();
    }
    
  #elif defined(USE_SERVOS2)
    int i, pin;
    for (i = 0; i < nServos; i++) {
      pin = myServoPins[i];
      myServos[pin].moveServo();
    }
  #endif
}

