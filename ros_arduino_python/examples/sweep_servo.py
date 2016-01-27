#!/usr/bin/env python

''' sweep_servo.py - Version 1.0 2015-12-04

    Move a servo back and forth between two positions

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2015 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
'''

import rospy
import sys
import os
from std_msgs.msg import Float64
from ros_arduino_msgs.srv import SetSpeed
from sensor_msgs.msg import JointState
from math import radians

class SweepServo():
    def __init__(self):
        # Set a name for the node
        self.node_name = "sweep_servo"
        
        # Initialize the node
        rospy.init_node(self.node_name)
                
        # Set a shutdown function to clean up when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        # Name of the joint we want to control
        joint_name = rospy.get_param('~joint', 'head_pan_joint')
            
        if joint_name is None or joint_name == '':
            rospy.logino("Joint name for servo must be specified in parameter file.")
            os._exit(1)
            
        max_position = radians(rospy.get_param('/arduino/joints/' + str(joint_name) + '/max_position'))
        min_position = radians(rospy.get_param('/arduino/joints/' + str(joint_name) + '/min_position'))
        
        target_max = max_position
        target_min = min_position
                
        # How fast should we sweep the servo
        servo_speed = rospy.get_param('~servo_speed', 1.0) # rad/s
        
        # Time delay between between sweeps
        delay = rospy.get_param('~delay', 0)  # seconds
                
        # Create a publisher for setting the joint position
        joint_pub = rospy.Publisher('/' + joint_name + '/command', Float64, queue_size=5)
        
        self.joint_state = JointState()
        
        # Subscribe to the /joint_states topic so we know where the servo is positioned
        rospy.Subscriber('/joint_states', JointState, self.update_joint_state)
        
        # Wait for the /joint_state message to come online
        rospy.wait_for_message('/joint_states', JointState, 60)
        
        # Wait until we actually have a message
        while self.joint_state == JointState():
            rospy.sleep(0.1)
        
        # Wait for the set_speed service
        rospy.loginfo('Waiting for set_speed services...')
        try:
            rospy.wait_for_service('/' + joint_name + '/set_speed', 60)
            rospy.loginfo('Ready.')
        except:
            rospy.loginfo('Could not connect to service!')
            os._exit(1)

        # Create a proxy for the set speed service
        set_speed = rospy.ServiceProxy('/' + joint_name + '/set_speed', SetSpeed)
        
        # Set the initial servo speed
        set_speed(servo_speed)

        rospy.loginfo('Sweeping servo...')
        
        while not rospy.is_shutdown():            
            while abs(self.get_joint_position(joint_name) - target_max) > 0.1:
                joint_pub.publish(target_max)
                rospy.sleep(0.1)
                
            rospy.sleep(delay)
            
            while abs(self.get_joint_position(joint_name) - target_min) > 0.1:  
                joint_pub.publish(target_min)
                rospy.sleep(0.1)
            
            rospy.sleep(delay)
    
    def update_joint_state(self, msg):
        self.joint_state = msg
        
    def get_joint_position(self, joint_name):
        index = self.joint_state.name.index(joint_name)
        return self.joint_state.position[index]
    
    def shutdown(self):
        rospy.loginfo('Shutting down ' + str(self.node_name))
        os._exit(0)
        
if __name__ == '__main__':
    try:
        SweepServo()
    except: 
        rospy.loginfo('Unexpected error: ' +  str(sys.exc_info()[0]))
        raise