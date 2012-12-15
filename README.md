This ROS stack includes an Arduino sketch and a collection of ROS
packages for controlling an Arduino-based robot using standard ROS
messages and services.

Supported sensors currently include Ping sonar and Sharp GP2D12
infrared as well as generic analog and digital sensors (e.g bump
switches, voltage sensors, etc.)

The package also includes a base controller for a differential drive
that accepts ROS Twist messages and publishes odometry data back to
the PC. In this version of the stack, the base controller requires the
use of an Arduino Mega 2560 controller together with a Pololu motor
controller shield (http://www.pololu.com/catalog/product/2502) and a
Robogaia Mega Encoder shield
(http://www.robogaia.com/two-axis-encoder-counter-mega-shield-version-2.html).

System Requirements
-------------------
The current version of the stack requires an Arudino Mega controller +
Pololu Dual VNH5019 Motor Driver Shield + Robogaia Encoder shield.  If
you do not have this hardware, you can still try the package for
reading sensors and controlling servos using other Arduino-compatible
controllers.  See the NOTES section at the end of this document for
instructions on how to do this.

To use the base controller you must also install the Polulu Arduino
library found at:

https://github.com/pololu/Dual-VNH5019-Motor-Shield

and the Robogaia Encoder library found at:

http://www.robogaia.com/uploads/6/8/0/9/6809982/__megaencodercounter-1.3.tar.gz

These libraries should be installed in your standard Arduino
sketchbook/libraries directory.

Finally, it is assumed you are using version 1.0 or greater of the
Arduino IDE.


Installation
------------

    $ cd ~/ros_workspace
    $ git clone https://github.com/hbrobotics/ros_arduino_bridge.git
    $ cd ros_arduino_bridge
    $ rosmake

The provided Arduino sketch is called MegaRobogaiaPololu and is
located in the ros\_arduino\_firmware package.  This sketch is
specific to the hardware requirements above but it can also be used
with other Arduino-type boards (e.g. Uno) by turning off the base
controller as described in the NOTES section at the end of this
document.

To install the MegaRobogaiaPololu sketch, follow these steps:

    $ cd SKETCHBOOK_PATH

where SKETCHBOOK_PATH is the path to your Arduino sketchbook directory.

    $ ln -s `rospack find ros_arduino_firmware`/src/libraries/MegaRobogaiaPololu MegaRobogaiaPololu

This last command creates a link in your sketchbook folder for the
MegaRobogaiaPololu sketch that you must run on your Arduino Mega
controller.  By creating a link rather than copying the files, the
sketch will get updated along with other files in the
ros\_arduino\_bridge stack.


Loading the MegaRobogaiaPololu Sketch
-------------------------------------

* Make sure you have already installed the DualVNH5019MotorShield and
  MegaEncoderCounter libraries into your Arduino sketchbook/libraries
  folder.

* Launch the Arduino 1.0 IDE and load the MegaRobogaiaPololu sketch.
  You should be able to find it by going to:

    File->Sketchbook->MegaRobogaiaPololu
  
NOTE: If you don't have the Arduino Mega/Pololu/Robogaia hardware but
still want to try the code, see the notes at the end of the file.

If you want to control PWM servos attached to your controller, change
the two lines that look like this:

<pre>
//#define USE_SERVOS
#undef USE_SERVOS
</pre>

to this:

<pre>
#define USE_SERVOS
//#undef USE_SERVOS
</pre>

You must then edit the include file servos.h and change the N_SERVOS
parameter as well as the pin numbers for the servos you have attached.

* Compile and upload the sketch to your Arduino.

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
port: /dev/ttyACM0
baud: 57600
timeout: 0.1

rate: 50
sensorstate_rate: 10

use_base_controller: True
base_controller_rate: 10

# === Robot drivetrain parameters
#wheel_diameter: 0.146
#wheel_track: 0.2969
#encoder_resolution: 8384 # from Pololu for 131:1 motors
#gear_reduction: 1.0
#motors_reversed: True

# ===  PID parameters
#Kp: 20
#Kd: 12
#Ki: 0
#Ko: 50

#  === Sensor definitions.  Examples only - edit for your robot.
#      Sensor type can be one of the follow (case sensitive!):
#	    Ping
#	    GP2D12
#	    Analog
#	    Digital
#	    PololuMotorCurrent

sensors: {
  motor_current_left:   {pin: 0, type: PololuMotorCurrent, rate: 5},
  motor_current_right:  {pin: 1, type: PololuMotorCurrent, rate: 5},
  ir_front_center:      {pin: 2, type: GP2D12, rate: 10},
  sonar_front_center:   {pin: 5, type: Ping, rate: 10},
  arduino_led:          {pin: 13, type: Digital, rate: 2, direction: output}
}
</pre>

Let's now look at each section of this file.

 _Port Settings_

The port will likely be either /dev/ttyACM0 or /dev/ttyUSB0 (e.g. for
an Xbee connection).  Set accordingly.

The MegaRobogaiaPololu Arudino sketch connects at 57600 baud by default.

_Polling Rates_

The main *rate* parameter (50 Hz by default) determines how fast the
outside ROS loop runs.  The default should suffice in most cases.  In
any event, it should be at least as fast as your fastest sensor rate
(defined below).

The *sensorstate\_rate* determines how often to publish an aggregated
list of all sensor readings.  Each sensor also publishes on its own
topic and rate.

The base\_controller\_rate determines how often to publish odometry readings.

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
the example above, the Arduino LED (pin 13) will be blinked on and off
at a rate of 2 times per second.

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
launch the ros\_arduiono\_python node with your parameters:

    $ roslaunch ros_arduino_python arduino.launch

You should see something like the following output:

<pre>
process[arduino-1]: started with pid [6098]
Connecting to Arduino on port /dev/ttyACM0 ...
Connected at 57600
Arduino is ready.
[INFO] [WallTime: 1355498525.954491] Connected to Arduino on port /dev/ttyACM0 at 57600 baud
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

    $ rostopic echo /arduino/sensors

To see the data on any particular sensor, echo its topic name:

    $ rostopic echo /arduino/sensor/sensor_name

For example, if you have a sensor called ir_front_center, you can see
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

ROS Services
------------
The ros_arduino_python package also defines a few ROS services as follows:

**digital\_set\_direction** - set the direction of a digital pin

    $ rosservice call /arduino/digital_set_direction pin direction

where pin is the pin number and direction is 0 for input and 1 for output.

**digital\_write** - send a LOW (0) or HIGH (1) signal to a digital pin

    $ rosservice call /arduino/digital_write pin value

where pin is the pin number and value is 0 for LOW and 1 for HIGH.

**servo\_write** - set the position of a servo

    $ rosservice call /arduino/servo_write id pos

where id is the index of the servo as defined in the Arduino sketch (servos.h) and pos is the position in degrees (0 - 180).

**servo\_read** - read the position of a servo

    $ rosservice call /arduino/servo_read id

where id is the index of the servo as defined in the Arduino sketch (servos.h)

NOTES
-----
If you do not have the hardware required to run the base controller,
follow the instructions below so that you can still use your
Arduino-compatible controller to read sensors and control PWM servos.

First, you need to edit the MegaRobogaiaPololu sketch. At the top of
the file, change the two lines that look like this:

<pre>
#define USE_BASE
//#undef USE_BASE
</pre>

to this:

<pre>
//#define USE_BASE
#undef USE_BASE
</pre>

You also need to comment out the line that looks like this:

    #include "MegaEncoderCounter.h"

so it looks like this:

    //#include "MegaEncoderCounter.h"

Compile the changes and upload to your controller.

Next, edit your my\_arduino_params.yaml file and set the
use\_base\_controller parameter to False.  That's all there is to it.