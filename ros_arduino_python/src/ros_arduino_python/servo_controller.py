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
from controllers import *

from math import radians, degrees

class Joint:
    def __init__(self, device, name):
        self.device = device
        self.name = name
        self.controller = None

        self.target_position = 0.0 # radians
        self.position = 0.0        # radians
        self.position_last = 0.0   # radians
        self.velocity = 0.0        # rad/s
    
class Servo(Joint):
    def __init__(self, device, name, ns="~joints"):
        Joint.__init__(self, device, name)
        
        # Construct the namespace for the joint
        n = ns + "/" + name + "/"
        
        # The Arduino pin used by this servo
        self.pin = int(rospy.get_param(n + "pin"))
        
        # Hobby servos have a rated speed giving in seconds per 60 degrees
        # A value of 0.24 seconds per 60 degrees is typical.
        self.rated_speed = rospy.get_param(n + "rated_speed", 0.24) # seconds per 60 degrees
        
        # Conversion factors to compute servo speed to step delay between updates
        self.max_rad_per_sec = radians(60.0) / self.rated_speed
        self.ms_per_rad = 1000.0 / self.max_rad_per_sec 
        
        # Convert initial servo speed in deg/s to a step delay in milliseconds
        step_delay = self.get_step_delay(radians(rospy.get_param(n + "init_speed", 90.0)))

        # Update the servo speed
        self.device.config_servo(self.pin, step_delay)

        # Min/max/neutral values
        self.neutral = radians(rospy.get_param(n + "neutral", 90.0))       # degrees
        self.max_position = radians(rospy.get_param(n + "max_position", 90.0))   # degrees
        self.min_position = radians(rospy.get_param(n + "min_position", -90.0))  # degrees
        self.range = radians(rospy.get_param(n + "range", 180))            # degrees
        self.max_speed = radians(rospy.get_param(n + "max_speed", 250.0))  # deg/s

        # Do we want to reverse positive motion
        self.invert = rospy.get_param(n + "invert", False)
        
        # Where to we want the servo positioned
        self.desired = self.neutral + radians(rospy.get_param(n + "init_position", 0))
                
        # Where is the servo positioned now
        self.position = 0.0
        
        # Subscribe to the servo's command topic for setting its position
        rospy.Subscriber("/" + name + '/command', Float64, self.command_cb)
        
        # Provide a number of services for controlling the servos
        rospy.Service(name + '/relax', Relax, self.relax_cb)
        rospy.Service(name + '/enable', Enable, self.enable_cb)
        rospy.Service(name + '/set_speed', SetSpeed, self.set_speed_cb)

    def command_cb(self, req):     
        # Check limits
        if req.data > self.max_position:
            req.data = self.max_position
        
        if req.data < self.min_position:
            req.data = self.min_position
        
        # Adjust for the neutral offset
        if self.invert:
            target_adjusted = self.neutral - req.data
        else:
             target_adjusted = self.neutral + req.data

        # Set the target position for the next servo controller update
        self.desired = target_adjusted
        
    def get_step_delay(self, target_speed=1.0):
        # Don't allow negative speeds
        target_speed = abs(target_speed)
        
        if target_speed > self.max_rad_per_sec:
            rospy.logdebug("Target speed exceeds max servo speed. Using max.")
            step_delay = 0
        else:
            # Catch division by zero and set to slowest speed possible
            try:
                step_delay = 1000.0 / degrees(1.0) * (1.0 / target_speed - 1.0 / self.max_rad_per_sec)
            except:
                step_delay = 32767
            
        # Minimum step delay is 0 millisecond
        step_delay = max(0, step_delay)
        
        return int(step_delay)
    
    def get_current_position(self):
        return self.device.servo_read(self.pin) - self.neutral
    
    def relax_cb(self, req):
        self.device.detach_servo(self.pin)
        
        return RelaxResponse()

    def enable_cb(self, req):
        if req.enable:
            self.device.attach_servo(self.pin)
        else:
            self.device.detach_servo(self.pin)
            
        return EnableResponse()
    
    def set_speed_cb(self, req):
        # Convert servo speed in rad/s to a step delay in milliseconds
        step_delay = self.get_step_delay(req.value)

        # Update the servo speed
        self.device.set_servo_delay(self.pin, step_delay)
        
        return SetSpeedResponse()

class ServoController(Controller):
    def __init__(self, device, name):
        Controller.__init__(self, device, name)

        self.servos = list()
        
        joint_update_rate = rospy.get_param("~joint_update_rate", 10.0)

        # Get the servo objects from the joint list
        for servo in self.device.joints.values():
            self.servos.append(servo)
            servo.position_last = servo.get_current_position()

        self.w_delta = rospy.Duration(1.0 / joint_update_rate)
        self.w_next = rospy.Time.now() + self.w_delta

        self.r_delta = rospy.Duration(1.0 / joint_update_rate)
        self.r_next = rospy.Time.now() + self.r_delta

    def poll(self):     
        """ Read and write servo positions and velocities. """
        if rospy.Time.now() > self.r_next:
            for servo in self.servos:
                servo.position = servo.get_current_position()
                
                # Compute velocity
                servo.velocity = (servo.position - servo.position_last) / self.r_delta.to_sec()
                servo.position_last = servo.position
                 
            self.r_next = rospy.Time.now() + self.r_delta   
        
        if rospy.Time.now() > self.w_next:
            for servo in self.servos: 
                self.device.servo_write(servo.pin, servo.desired)
            self.w_next = rospy.Time.now() + self.w_delta
            
            