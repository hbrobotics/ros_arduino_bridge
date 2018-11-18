For ROS Kinetic, Ubuntu 16.04 and Python2 (might work with ROS Indigo and Ubuntu 14.04)

Table of Contents
=================

* [Overview](#overview)
* [Official ROS Documentation](#official-ros-documentation)
* [System Requirements](#system-requirements)
* [Hardware Requirements](#hardware-requirements)
* [Preparing your Serial Port under Linux](#preparing-your-serial-port-under-linux)
* [Installation of the ros_arduino_bridge Stack](#installation-of-the-ros_arduino_bridge-stack)
* [Loading the ROSArduinoBridge Sketch](#loading-the-rosarduinobridge-sketch)
* [Firmware Commands](#firmware-commands)
* [Testing your Wiring Connections](#testing-your-wiring-connections)
* [Configuring the ros_arduino_python Node](#configuring-the-ros_arduino_python-node)
* [Launching the ros_arduino_python Node](#launching-the-ros_arduino_python-node)
* [Viewing Sensor Data](#viewing-sensor-data)
* [Sending Twist Commands and Viewing Odometry Data](#sending-twist-commands-and-viewing-odometry-data)
* [ROS Services for Sensors and Servos](#ros-services-for-sensors-and-servos)
* [ROS Joint Topics and Services](#ros-joint-topics-and-services)
* [Using the on-board wheel encoder counters (Arduino Uno only)](#using-the-on-board-wheel-encoder-counters-arduino-uno-only)
* [Running without a Base Controller](#running-without-a-base-controller)

Overview
--------
This branch (kinetic-devel) is intended for ROS Kinetic and above, and uses the catkin buildsystem. It may also be compatible with ROS Indigo.

This ROS metapackage includes an Arduino library (called ROSArduinoBridge) and a collection of ROS packages for controlling an Arduino-based robot using standard ROS messages and services.  The stack does **not** depend on ROS Serial.

Features of the package include:

* Direct support for Ping sonar and Sharp infrared (GP2D12) sensors

* Can also read data from generic analog and digital sensors

* Can control digital outputs (e.g. turn a switch or LED on and off)

* Support for PWM servos

* Configurable base controller for a differential drive mobile robot including support for encoders if using the required hardware (see [Hardware](#Hardware) below)

* ROS diagnostics messages published for each attached sensor

Official ROS Documentation
--------------------------
A standard ROS-style version of this documentation can be found on the ROS wiki at:

http://www.ros.org/wiki/ros_arduino_bridge


System Requirements
-------------------
**ROS Dependencies**

    $ sudo apt-get install ros-kinetic-diagnostic-updater ros-kinetic-control-msgs ros-kinetic-nav-msgs

**Python Serial:** To install the python-serial package under Ubuntu, use the command:

    $ sudo apt-get install python-serial

On non-Ubuntu systems, use either:

    $ sudo pip install --upgrade pyserial

or

    $ sudo easy_install -U pyserial

**Arduino IDE 1.6.6 or Higher:**
Note that the preprocessing of conditional #include statements is broken in earlier versions of the Arduino IDE.  To ensure that the ROS Arduino Bridge firmware compiles correctly, be sure to install version 1.6.6 or higher of the Arduino IDE.  You can download the IDE from https://www.arduino.cc/en/Main/Software.

Hardware Requirements
---------------------
The firmware should work with any Arduino-compatible controller for reading sensors and controlling PWM servos.  However, to use the base controller, you will need a supported motor controller and encoder hardware as described below.
If you do not have this hardware, you can still try the package for reading sensors and controlling servos.  See [Running without a Base Controller](#unning-without-a-base-controller) at the end of this document for instructions on how to do this.

The base controller requires the use of a motor controller and encoders for reading odometry data.  The current version of the package provides support for the following motor controller and encoder hardware:

* Robogaia 3-axis Encoder shield (Compatible with Arduino Mega, Due, Uno)
(https://www.robogaia.com/3-axis-encoder-conter-arduino-shield.html)

* Mega Moto Controller (http://www.robotshop.com/en/arduino-compatible-mega-motor-shield-1a-5-28v.html)

* Arduino R3 Motor Controller (http://arduino.cc/en/Main/ArduinoMotorShieldR3 [without the brake capability])

* Instead of the Encoder shield, wheel encoders can be [connected directly](#using-the-on-board-wheel-encoder-counters-arduino-uno-only) (**NOTE:** The on-board wheel encoder counters are currently only supported by Arduino Uno.)

* The library can be easily extended to include support for other motor controllers and encoder hardware or libraries.

To use the base controller you must also install the appropriate libraries for your motor controller and encoders.

The Robogaia Mega Encoder library can be found at:

http://www.robogaia.com/uploads/6/8/0/9/6809982/__megaencodercounter-1.3.tar.gz

Please see the website for the supported motor controllers above for library support.

These libraries should be installed in your standard Arduino
sketchbook/libraries directory.

Finally, it is assumed you are using version 1.6.6 or greater of the
Arduino IDE.

Preparing your Serial Port under Linux
--------------------------------------
Your Arduino will likely connect to your Linux computer as port /dev/ttyACM# or /dev/ttyUSB# where # is a number like 0, 1, 2, etc., depending on how many other devices are connected.  The easiest way to make the determination is to unplug all other USB devices, plug in your Arduino, then run the command:

    $ ls /dev/ttyACM*

or 

    $ ls /dev/ttyUSB*

Hopefully, one of these two commands will return the result you're looking for (e.g. /dev/ttyACM0) and the other will return the error "No such file or directory".

Next you need to make sure you have read/write access to the port.  Assuming your Arduino is connected on /dev/ttyACM0, run the command:

    $ ls -l /dev/ttyACM0

and you should see an output similar to the following:

    crw-rw---- 1 root dialout 166, 0 2018-02-24 08:31 /dev/ttyACM0

Note that only root and the "dialout" group have read/write access.  Therefore, you need to be a member of the dialout group.  You only have to do this once and it should then work for all USB devices you plug in later on.

To add yourself to the dialout group, run the command:

    $ sudo usermod -a -G dialout your_user_name

where your\_user\_name is your Linux login name.  You will likely have to log out of your X-window session then log in again, or simply reboot your machine if you want to be sure.

When you log back in again, try the command:

    $ groups

and you should see a list of groups you belong to including dialout. 

Installation of the ros\_arduino\_bridge Package
----------------------------------------------

    $ cd ~/catkin_workspace/src
    $ git clone https://github.com/hbrobotics/ros_arduino_bridge.git
    $ cd ~/catkin_workspace
    $ catkin_make

The provided Arduino library is called ROSArduinoBridge and is
located in the ros\_arduino\_firmware package.  This sketch is
specific to the hardware requirements above but it can also be used
with other Arduino-type boards (e.g. Uno) by turning off the base
controller as described in the NOTES section at the end of this
document.

To install the ROSArduinoBridge library, follow these steps:

    $ cd SKETCHBOOK_PATH

where SKETCHBOOK_PATH is the path to your Arduino sketchbook directory.

    $ \cp -rp  `rospack find ros_arduino_firmware`/src/libraries/ROSArduinoBridge -T ROSArduinoBridge

This last command copies the ROSArduinoBridge sketch files into your sketchbook folder and overwrites any existing files with the same name.  The next section describes how to configure, compile and upload this sketch.


Loading the ROSArduinoBridge Sketch
-----------------------------------

* If you are using the base controller, make sure you have already installed the appropriate motor controller and encoder libraries into your Arduino sketchbook/librariesfolder.

* Launch the Arduino IDE and load the ROSArduinoBridge sketch.
  You should be able to find it by going to:

    File->Sketchbook->ROSArduinoBridge
  
**NOTE:** If you have the required hardware to use the base controller, uncomment the line that looks like this:

<pre>
//#define USE_BASE
</pre>

so it looks like this:

<pre>
#define USE_BASE
</pre>

You will also need to choose one of the supported motor controllers by uncommenting its #define statement and commenting out any others.  By default, the Adafruit Motor Driver v2.3 is chosen.

Choose a supported encoder library by by uncommenting its #define statement and commenting out any others.  At the moment, the two options are the Robogaia Mega Encoder shield (chosen by default) and the directo connection ARDUINO_ENC_COUNTER option that works for Arduino Uno compatible boards.

By default, the sketch will provide support to control PWM servos attached to your Arduino.  If you do not need servo support, you can comment out the line that looks like this:

<pre>
#define USE_SERVOS2
</pre>

so that it looks like this:

<pre>
//#define USE_SERVOS2
</pre>


* Compile and upload the sketch to your Arduino.

Firmware Commands
-----------------
The ROSArduinoLibrary accepts single-letter commands over the serial port for polling sensors, controlling servos, driving the robot, and reading encoders.  These commands can be sent to the Arduino over any serial interface, including the Serial Monitor in the Arduino IDE.

**NOTE:** Before trying these commands, set the Serial Monitor baudrate to 57600 and the line terminator to "Carriage return" or "Both NL & CR" using the two pulldown menus on the lower right of the Serial Monitor window.

The list of commands can be found in the file commands.h.  The current list includes:

<pre>
#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define CONFIG_SERVO   'j'
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define SERVO_DELAY    'v'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define ATTACH_SERVO   'y'
#define DETACH_SERVO   'z'
#define LEFT            0
#define RIGHT           1
</pre>

For example, to get the analog reading on pin 3, use the command:

a 3

To change the mode of digital pin 3 to OUTPUT, send the command:

c 3 1

To get the current encoder counts:

e

To move the robot forward at 20 encoder ticks per second:

m 20 20

To intialize a PWM servo on pin 3 with speed delay 100ms:

j 3 100

To move the servo on pin 3 to position 120 degrees:

s 3 120

To detach servo on pin 3:

z 3


Testing your Wiring Connections
-------------------------------
On a differential drive robot, the motors are connected to the motor controller terminals with opposite polarities to each other.  Similarly, the A/B leads from the encoders are connected in the reverse sense to each other.  However, you still need to make sure that (a) the wheels move forward when given a positive motor speed and (b) that the encoder counts increase when the wheels move forward.

After **placing your robot on blocks**, you can use the Serial Monitor in the Arduino IDE to test both requirements.  Use the 'm' command to activate the motors, the 'e' command to get the encoder counts, and the 'r' command to reset the encoders to 0.  Remember that at the firmware level, motor speeds are given in encoder ticks per second so that for an encoder resolution of, say 4000 counts per wheel revolution, a command such as 'm 20 20' should move the wheels fairly slowly.  (The wheels will only move for 2 seconds which is the default setting for the AUTO\_STOP\_INTERVAL.)  Also remember that the first argument is the left motor speed and the second argument is the right motor speed.  Similarly, when using the 'e' command, the first number returned is the left encoder count and the second number is the right encoder count.

Finally, you can use the 'r' and 'e' commands to verify the expected encoder counts by rotating the wheels by hand roughly one full turn and checking the reported counts.


Configuring the ros\_arduino\_python Node
-----------------------------------------
Now that your Arduino is running the required sketch, you can
configure the ROS side of things on your PC.  You define your robot's
dimensions, PID parameters, and sensor configuration by editing the
YAML file in the directory ros\_arduino\_python/config.  So first move
into that directory:

    $ roscd ros_arduino_python/config

Now copy the provided config file to one you can modify:

    $ cp arduino_params.yaml my_arduino_params.yaml

Bring up your copy of the params file (my\_arduino\_params.yaml) in
your favorite text editor.  It should start off looking like this:

<pre>
port: /dev/ttyUSB0
baud: 57600
timeout: 0.1

rate: 50
sensorstate_rate: 10

use_base_controller: False
base_controller_rate: 10

# === Robot drivetrain parameters
#wheel_diameter: 0.146
#wheel_track: 0.2969
#encoder_resolution: 8384 # from Pololu for 131:1 motors
#gear_reduction: 1.0
#motors_reversed: True

# === PID parameters
#Kp: 20
#Kd: 12
#Ki: 0
#Ko: 50
#accel_limit: 1.0

# === Sensor definitions.  Examples only - edit for your robot.
#     Sensor type can be one of the follow (case sensitive!):
#	  * Ping
#	  * GP2D12
#	  * Analog
#	  * Digital
#	  * PololuMotorCurrent
#	  * PhidgetsVoltage
#	  * PhidgetsCurrent (20 Amp, DC)

sensors: {
  #motor_current_left:   {pin: 0, type: PololuMotorCurrent, rate: 5},
  #motor_current_right:  {pin: 1, type: PololuMotorCurrent, rate: 5},
  #ir_front_center:      {pin: 2, type: GP2D12, rate: 10},
  #sonar_front_center:   {pin: 5, type: Ping, rate: 10},
  onboard_led:           {pin: 13, type: Digital, rate: 5, direction: output}
}

# Joint name and configuration is an example only
joints: {
    head_pan_joint: {pin: 3, init_position: 0, init_speed: 90, neutral: 90, min_angle: -90, max_angle: 90, invert: False, continous: False},
    head_tilt_joint: {pin: 5, init_position: 0, init_speed: 90, neutral: 90, min_angle: -90, max_angle: 90, invert: False, continous: False}
}


</pre>

**NOTE**: Do not use tabs in your .yaml file or the parser will barf it back out when it tries to load it.   Always use spaces instead.  **ALSO**: When defining your sensor parameters, the last sensor in the list does **not** get a comma (,) at the end of the line but all the rest **must** have a comma.

Let's now look at each section of this file.

 _Port Settings_

The port will likely be either /dev/ttyACM0 or /dev/ttyUSB0. Set accordingly.

The MegaRobogaiaPololu Arudino sketch connects at 57600 baud by default.

_Polling Rates_

The main *rate* parameter (50 Hz by default) determines how fast the
outside ROS loop runs.  The default should suffice in most cases.  In
any event, it should be at least as fast as your fastest sensor rate
(defined below).

The *sensorstate\_rate* determines how often to publish an aggregated
list of all sensor readings.  Each sensor also publishes on its own
topic and rate.

The *use\_base\_controller* parameter is set to False by default.  Set it to True to use base control (assuming you have the required hardware.)  You will also have to set the PID paramters that follow.

The *base\_controller\_rate* determines how often to publish odometry readings.

_Defining Sensors_

The *sensors* parameter defines a dictionary of sensor names and
sensor parameters. (You can name each sensor whatever you like but
remember that the name for a sensor will also become the topic name
for that sensor.)

The four most important parameters are *pin*, *type*, *rate* and *direction*.
The *rate* defines how many times per second you want to poll that
sensor.  For example, a voltage sensor might only be polled once a
second (or even once every 2 seconds: rate=0.5), whereas a sonar
sensor might be polled at 20 times per second.  The *type* must be one
of those listed (case sensitive!).  The default *direction* is input so
to define an output pin, set the direction explicitly to output.  In
the example above, the Arduino LED (pin 13) will be turned on and off
at a rate of 2 times per second.

_Defining Servo Configurations_

The *joints* parameter defines a dictionary of joint names and servo parameters.  (You can name each joint whatever you like but rememember that joint names will become part of the servo's ROS topic and service names.)

The most important parameter is *pin* which of course must match the pin the servo attaches to on your Arduino.  Most PWM servos operate from 0 to 180 degrees with a "neutral" point of 90 degrees. ROS uses radians instead of degrees for joint positions but it is usually easier for programmers to specify the angular limits in the config file using degrees.  The ROS Arduino Bridge pacakge takes care of the conversion to radians.  An *init_position* of 0 therefore means 0 degrees relative to the neutral point of 90 degrees.  A *max_angle* of 90 degrees maps into 180 degrees at the servo. 

_Setting Drivetrain and PID Parameters_

To use the base controller, you will have to uncomment and set the
robot drivetrain and PID parameters.  The sample drivetrain parameters
are for 6" drive wheels that are 11.5" apart.  Note that ROS uses
meters for distance so convert accordingly.  The sample encoder
resolution (ticks per revolution) is from the specs for the Pololu
131:1 motor.  Set the appropriate number for your motor/encoder
combination.  Set the motors_reversed to True if you find your wheels
are turning backward, otherwise set to False.

The PID parameters are trickier to set.  You can start with the sample
values but be sure to place your robot on blocks before sending it
your first Twist command.

Launching the ros\_arduino\_python Node
---------------------------------------
Take a look at the launch file arduino.launch in the
ros\_arduino\_python/launch directory.  As you can see, it points to a
config file called my\_arduino\_params.yaml.  If you named your config
file something different, change the name in the launch file.

With your Arduino connected and running the MegaRobogaiaPololu sketch,
launch the ros\_arduino\_python node with your parameters:

    $ roslaunch ros_arduino_python arduino.launch

You should see something like the following output:

<pre>
process[arduino-1]: started with pid [6098]
Connecting to Arduino on port /dev/ttyUSB0 ...
Connected at 57600
Arduino is ready.
[INFO] [WallTime: 1355498525.954491] Connected to Arduino on port /dev/ttyUSB0 at 57600 baud
[INFO] [WallTime: 1355498525.966825] motor_current_right {'rate': 5, 'type': 'PololuMotorCurrent', 'pin': 1}
[INFO]
etc
</pre>

If you have any Ping sonar sensors on your robot and you defined them
in your config file, they should start flashing to indicate you have
made the connection.

Viewing Sensor Data
-------------------
To see the aggregated sensor data, echo the sensor state topic:

    $ rostopic echo /arduino/sensor_state

To see the data on any particular sensor, echo its topic name:

    $ rostopic echo /arduino/sensor/sensor_name

For example, if you have a sensor called ir\_front\_center, you can see
its data using:

    $ rostopic echo /arduino/sensor/ir_front_center

You can also graph the range data using rxplot:

    $ rxplot -p 60 /arduino/sensor/ir_front_center/range


Sending Twist Commands and Viewing Odometry Data
------------------------------------------------

Place your robot on blocks, then try publishing a Twist command:

    $ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{ angular: {z: 0.5} }'

The wheels should turn in a direction consistent with a
counter-clockwise rotation (right wheel forward, left wheel backward).
If they turn in the opposite direction, set the motors_reversed
parameter in your config file to the opposite of its current setting,
then kill and restart the arduino.launch file.

Stop the robot with the command:

    $ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'

To view odometry data:

    $ rostopic echo /odom

or

   $ rxplot -p 60 /odom/pose/pose/position/x:y, /odom/twist/twist/linear/x, /odom/twist/twist/angular/z

ROS Services for Sensors and Servos
-----------------------------------
The ros\_arduino\_python package also defines a few ROS services for sensors and servos as follows:

**digital\_set\_direction** - set the direction of a digital pin

    $ rosservice call /arduino/digital_set_direction pin direction

where pin is the pin number and direction is 0 for input and 1 for output.

**digital\_write** - send a LOW (0) or HIGH (1) signal to a digital pin

    $ rosservice call /arduino/digital_write pin value

where pin is the pin number and value is 0 for LOW and 1 for HIGH.

**servo\_write** - set the position of a servo

    $ rosservice call /arduino/servo_write id pos

where id is the index of the servo as defined in the Arduino sketch (servos.h) and pos is the position in radians (0 - 3.14).

**servo\_read** - read the position of a servo

    $ rosservice call /arduino/servo_read id

where id is the index of the servo as defined in the Arduino sketch (servos.h)

ROS Joint Topics and Services
-----------------------------
At the ROS level, a servo is called a joint and each joint has its own topics and services.  To change the position of a joint, publish the position
in radians to the topic:

**/\<joint_name\>/command**

For example, a joint called head_pan_joint in the YAML config file can be controlled using the topic:

**/head_pan_joint/command**

which takes a Float64 argument specifying the desired position in radians.  For example, the command:

    $ rostopic pub -1 /head_pan_joint/command std_msgs/Float64 -- 1.0

will move the servo to angle 1.0 radians from the neutral point; i.e. about 147 degrees when using the default neutral point of 90 degrees.  Using a negative value moves the servo in the other direction:

    $ rostopic pub -1 /head_pan_joint/command std_msgs/Float64 -- -1.0

A number of services are also available for each joint:

**/\<joint_name\>/enable** - Enable or disable a joint.  Disabling also detachs the underlying servo so that it can be moved by hand.

    $ rosservice call /head_pan_joint/enable false

**/\<joint_name\>/relax** - Another way to detach the underlying servo so that it can be moved by hand.

    $ rosservice call /head_pan_joint/relax

**/\<joint_name\>/set_speed** - Set the movement speed of servo in radians per second.

    $ rosservice call /head_pan_joint/set_speed 1.0


Using the on-board wheel encoder counters (Arduino Uno only)
------------------------------------------------------------
The firmware supports on-board wheel encoder counters for Arduino Uno.
This allows connecting wheel encoders directly to the Arduino board, without the need for any additional wheel encoder counter equipment (such as a RoboGaia encoder shield).

For speed, the code is directly addressing specific Atmega328p ports and interrupts, making this implementation Atmega328p (Arduino Uno) dependent. (It should be easy to adapt for other boards/AVR chips though.)

To use the on-board wheel encoder counters, connect your wheel encoders to Arduino Uno as follows:

    Left wheel encoder A output -- Arduino UNO pin 2
    Left wheel encoder B output -- Arduino UNO pin 3

    Right wheel encoder A output -- Arduino UNO pin A4
    Right wheel encoder B output -- Arduino UNO pin A5

Make the following changes in the ROSArduinoBridge sketch to disable the RoboGaia encoder shield, and enable the on-board one:

    /* The RoboGaia encoder shield */
    //#define ROBOGAIA
    /* Encoders directly attached to Arduino board */
    #define ARDUINO_ENC_COUNTER

Compile the changes and upload to your controller.


Running without a Base Controller
---------------------------------
If you do not have the hardware required to run the base controller,
follow the instructions below so that you can still use your
Arduino-compatible controller to read sensors and control PWM servos.

First, you need to edit the ROSArduinoBridge sketch. At the top of
the file, comment out the line that looks like this:

<pre>
#define USE_BASE
</pre>

so it looks like this:

<pre>
//#define USE_BASE
</pre>

(You may find that it is already commented out.)

**NOTE:** If you are using a version of the Arduino IDE earlier than 1.6.6, then you also need to comment out the line that looks like this in the file encoder_driver.ino:

    #include "MegaEncoderCounter.h"

so it looks like this:

    //#include "MegaEncoderCounter.h"

Compile the changes and upload to your controller.

Next, edit your my\_arduino_params.yaml file and make sure the
use\_base\_controller parameter is set to False.  That's all there is to it.
