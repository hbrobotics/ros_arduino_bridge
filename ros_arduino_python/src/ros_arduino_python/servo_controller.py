#!/usr/bin/env python

"""
    Servo class for the arudino_python package
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2015 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
    
    Borrowed heavily from Mike Feguson's ArbotiX servos_controller.py code.
"""
import rospy
from std_msgs.msg import Float64
from ros_arduino_msgs.srv import Relax, Enable, SetSpeed, SetSpeedResponse, RelaxResponse, EnableResponse
from ros_arduino_python.diagnostics import DiagnosticsUpdater
from ros_arduino_python.arduino_driver import CommandErrorCode, CommandException
from controllers import *

from math import radians, degrees, copysign

class Joint:
    def __init__(self, device, name):
        self.device = device
        self.name = name

        self.position = 0.0        # radians
        self.start_position = 0.0
        self.position_last = 0.0   # radians
        self.velocity = 0.0        # rad/s
        self.servo_speed = 0.0     # rad/s
        self.direction = 1
        self.is_moving = False
        self.in_trajectory = False

        # Stamp the start of the movement
        self.time_move_started = rospy.Time.now()

        # Joint update rates are all the same joint_update_rate parameter
        self.rate = self.device.joint_update_rate

        # Track diagnostics for this component
        diagnotics_error_threshold = self.get_kwargs('diagnotics_error_threshold', 10)
        diagnostics_rate = float(self.get_kwargs('diagnostics_rate', 1))

        # The DiagnosticsUpdater class is defined in the diagnostics.py module
        self.diagnostics = DiagnosticsUpdater(self, name + '_joint', diagnotics_error_threshold, diagnostics_rate)

    def get_kwargs(self, arg, default):
        try:
            return kwargs['arg']
        except:
            return default

class Servo(Joint):
    def __init__(self, device, name, ns="~joints"):
        Joint.__init__(self, device, name)
        
        # Construct the namespace for the joint
        namespace = ns + "/" + name + "/"
        
        # The Arduino pin used by this servo
        self.pin = int(rospy.get_param(namespace + "pin"))
        
        # Hobby servos have a rated speed giving in seconds per 60 degrees
        # A value of 0.24 seconds per 60 degrees is typical.
        self.rated_speed = rospy.get_param(namespace + 'rated_speed', 0.24) # seconds per 60 degrees

        # Convert rated speed to degrees per second
        self.rated_speed_deg_per_sec = 60.0 / self.rated_speed

        # Convert rated speed to radians per second
        self.rated_speed_rad_per_sec = radians(self.rated_speed_deg_per_sec)

        # The rated speed might require an emperical correction factor
        self.joint_speed_scale_correction = rospy.get_param(namespace + 'joint_speed_scale_correction', 1.0)

        # Get the initial servo speed in degrees per second
        self.servo_speed = radians(rospy.get_param(namespace + 'init_speed', 60.0))

        self.direction = copysign(1, -self.servo_speed)

        # Convert initial servo speed in rad/s to a step delay in milliseconds
        step_delay = self.get_step_delay(self.servo_speed)

        # Enable the servo
        self.device.attach_servo(self.pin)

        # Set the servo speed
        self.device.set_servo_delay(self.pin, step_delay)

        # Min/max/neutral values
        self.neutral = rospy.get_param(namespace + 'neutral', 90.0)                     # degrees
        self.max_position = radians(rospy.get_param(namespace + 'max_position', 90.0))  # degrees to radians
        self.min_position = radians(rospy.get_param(namespace + 'min_position', -90.0)) # degrees to radians
        self.range = radians(rospy.get_param(namespace + 'range', 180))                 # degrees to radians
        self.max_speed = radians(rospy.get_param(namespace + 'max_speed', 250.0))       # deg/s to rad/s

        # Do we want to reverse positive motion
        self.invert = rospy.get_param(namespace + 'invert', False)
        
        # Intialize the desired position of the servo from the init_position parameter
        self.desired = radians(rospy.get_param(namespace + 'init_position', 0))

        # Where is the servo positioned now
        #self.position = self.desired

        # Subscribe to the servo's command topic for setting its position
        rospy.Subscriber('/' + name + '/command', Float64, self.command_cb)

        # Provide a number of services for controlling the servos
        rospy.Service(name + '/relax', Relax, self.relax_cb)
        rospy.Service(name + '/enable', Enable, self.enable_cb)
        rospy.Service(name + '/set_speed', SetSpeed, self.set_speed_cb)

    def command_cb(self, msg):
        # Check limits
        if msg.data > self.max_position:
            msg.data = self.max_position

        if msg.data < self.min_position:
            msg.data = self.min_position

        # Adjust for the neutral offset
        if self.invert:
            target_adjusted = self.neutral - msg.data
        else:
            target_adjusted = self.neutral + msg.data

        # Stamp the start of the movement
        self.time_move_started = rospy.Time.now()

        # Record the starting position
        self.start_position = self.get_current_position()

        # Set the target position for the next servo controller update
        self.desired = msg.data

        # Flag to indicate the servo is moving
        self.is_moving = True

        # The direction of movement
        self.direction = copysign(1, self.desired - self.position)

    def get_step_delay(self, target_speed=1.0):
        # Don't allow negative speeds
        target_speed = abs(target_speed)

        if target_speed > self.rated_speed_rad_per_sec:
            rospy.logdebug("Target speed exceeds max servo speed. Using max.")
            step_delay = 0
        else:
            # Catch division by zero and set to slowest speed possible
            try:
                step_delay = 1000.0 * radians(1.0) * ((1.0 / (self.joint_speed_scale_correction * target_speed)) - (1.0 / self.rated_speed_rad_per_sec))
            except:
                step_delay = 4294967295 # 2^32 - 1
                            
        # Minimum step delay is 0 millisecond
        step_delay = max(0, step_delay)

        return int(step_delay)

    def radians_to_servo_degrees(self, rad):
        if self.invert:
            servo_degrees = self.neutral - degrees(rad)
        else:
            servo_degrees = degrees(rad) + self.neutral

        return servo_degrees

    def get_current_position(self):
        #return radians(self.device.servo_read(self.pin)) + self.neutral
        return radians(self.device.servo_read(self.pin) - self.neutral)

    def get_interpolated_position(self):
        time_since_start = rospy.Time.now() - self.time_move_started
    
        return self.start_position + self.servo_speed * self.direction * time_since_start.to_sec()

    def relax_cb(self, req):
        self.device.detach_servo(self.pin)

        return RelaxResponse()

    def enable_cb(self, req):
        if req.enable:
            self.device.attach_servo(self.pin)
            state = True
        else:
            self.device.detach_servo(self.pin)
            state = False

        return EnableResponse(state)

    def set_speed_cb(self, req):
        # Convert servo speed in rad/s to a step delay in milliseconds
        step_delay = self.get_step_delay(req.speed)

        # Update the servo speed
        self.device.set_servo_delay(self.pin, step_delay)

        self.servo_speed = req.speed

        return SetSpeedResponse()

class ContinousServo(Joint):
    def __init__(self, device, name, ns="~joints"):
        Joint.__init__(self, device, name)

        # Construct the namespace for the joint
        namespace = ns + "/" + name + "/"

        # The Arduino pin used by this servo
        self.pin = int(rospy.get_param(namespace + "pin"))

        # Min/max/neutral values
        self.neutral = rospy.get_param(namespace + "neutral", 90.0)                # degrees
        self.max_speed = radians(rospy.get_param(namespace + "max_speed", 250.0))  # deg/s
        self.min_speed = radians(rospy.get_param(namespace + "min_speed", 0))      # deg/s
        self.range = radians(rospy.get_param(namespace + "range", 180))            # degrees

        # Do we want to reverse positive motion
        self.invert = rospy.get_param(namespace + "invert", False)

        # The initial speed of the servo
        self.init_speed = self.neutral + radians(rospy.get_param(namespace +  "init_speed", 0))

        # Conversion factors to compute servo speed in rad/s from control input
        self.max_rad_per_sec = radians(60.0) / self.rated_speed

        #self.ticks_per_rad_per_sec = (self.range / 2.0) / self.max_rad_per_sec

        # What is the current speed
        self.velocity = 0.0

        # Enable the servo
        self.device.attach_servo(self.pin)

        # Subscribe to the servo's command topic for setting its speed
        rospy.Subscriber("/" + name + '/command', Float64, self.command_cb)

    def command_cb(self, req):
        # Check limits
        if msg.data > self.max_position:
            msg.data = self.max_position

        if msg.data < self.min_position:
            msg.data = self.min_position

        # Adjust for the neutral offset
        if self.invert:
            target_adjusted = self.neutral - msg.data
        else:
            target_adjusted = self.neutral + msg.data

        # Stamp the start of the movement
        self.time_move_started = rospy.Time.now()

        # Record the starting position
        self.start_position = self.get_current_position()

        # Set the target position for the next servo controller update
        self.desired = msg.data

        self.is_moving = True
        self.direction = copysign(1, self.desired - self.position)

class ServoController():
    def __init__(self, device, name):
        self.name = name
        self.device = device
        self.servos = list()

        # Get the servo objects from the joint list
        for servo in self.device.joints.values():
            self.servos.append(servo)
            servo.position_last = servo.get_current_position()

        self.delta_t = rospy.Duration(1.0 / self.device.joint_update_rate)
        self.next_update = rospy.Time.now() + self.delta_t

    def poll(self):     
        """ Read and write servo positions and velocities. """
        if rospy.Time.now() > self.next_update:
            for servo in self.servos:

                
                # Check to see if we are within 2 degrees of the desired position or gone past it
                if not servo.in_trajectory and servo.is_moving:
                    if abs(servo.position - servo.desired) < radians(2.0) or copysign(1, servo.desired - servo.position) != servo.direction:
                        servo.position = servo.desired
                        servo.is_moving = False
                        servo.velocity = 0.0
                        duration =  rospy.Time.now() - servo.time_move_started
                        
                # If the servo is still moving, update its interpolated position and velocity
                if servo.is_moving:
                    try:
                        # Get the interpolated current position of the servo
                        servo.position = servo.get_interpolated_position()
                        if not servo.in_trajectory:
                            servo.position = servo.get_interpolated_position()
                        else:
                            servo.position = servo.get_current_position()

                        # We cannot interpolate both position and velocity so just set velocity to the current speed
                        servo.velocity = servo.servo_speed
                        servo.position_last = servo.position
                        
                        # Update diagnostics counters
                        servo.diagnostics.reads += 1
                        servo.diagnostics.total_reads += 1

                        # Add a successful poll to the frequency status diagnostic task
                        servo.diagnostics.freq_diag.tick()
                    except (CommandException, TypeError) as e:
                        # Update error counter
                        servo.diagnostics.errors += 1
                        rospy.logerr('Command Exception: ' + CommandErrorCode.ErrorCodeStrings[e.code])
                        rospy.logerr("Invalid value read from joint: " + str(self.name))

                    try:
                        # Send desired position to the servo and update diagnostics
                        self.device.servo_write(servo.pin, servo.radians_to_servo_degrees(servo.desired))
                        servo.diagnostics.reads += 1
                        servo.diagnostics.total_reads += 1
                    except (CommandException, TypeError) as e:
                        servo.diagnostics.errors += 1
                        rospy.logerr('Command Exception: ' + CommandErrorCode.ErrorCodeStrings[e.code])
                        rospy.logerr("Invalid value read from joint: " + str(self.name))

            self.next_update = rospy.Time.now() + self.delta_t
        
