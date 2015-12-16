#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from ros_arduino_python.arduino_driver import Arduino
from ros_arduino_python.arduino_sensors import *
from ros_arduino_msgs.srv import *
from ros_arduino_python.base_controller import BaseController
from ros_arduino_python.servo_controller import Servo, ServoController
from ros_arduino_python.follow_controller import FollowController
from ros_arduino_python.joint_state_publisher import JointStatePublisher
from geometry_msgs.msg import Twist
import os, time
import thread
from math import radians

controller_types = { "follow_controller" : FollowController }

class ArduinoROS():
    def __init__(self):
        rospy.init_node('Arduino', log_level=rospy.INFO)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        
        self.use_base_controller = rospy.get_param("~use_base_controller", False)
        
        # Assume we don't have any joints by default
        self.have_joints = False
        
        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # The SensorState publisher periodically publishes the values of all sensors on
        # a single topic.
        self.sensorStatePub = rospy.Publisher('~sensor_state', SensorState, queue_size=5)
        
        # A service to position a PWM servo
        rospy.Service('~servo_write', ServoWrite, self.ServoWriteHandler)
        
        # A service to read the position of a PWM servo
        rospy.Service('~servo_read', ServoRead, self.ServoReadHandler)
        
        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)
        
        # A service to turn a digital sensor on or off
        rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)
        
        # A service to read the value of a digital sensor
        rospy.Service('~digital_read', DigitalRead, self.DigitalReadHandler) 

        # A service to set pwm values for the pins
        rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)
        
        # A service to read the value of an analog sensor
        rospy.Service('~analog_read', AnalogRead, self.AnalogReadHandler)

        # Initialize the device
        self.device = Arduino(self.port, self.baud, self.timeout)
        
        # Make the connection
        self.device.connect()
        
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = thread.allocate_lock()

        # Initialize any sensors
        self.mySensors = list()
        
        # Read in the sensors parameter dictionary
        sensor_params = rospy.get_param("~sensors", dict({}))
        
        # Initialize individual sensors appropriately
        for name, params in sensor_params.iteritems():
            # Set the direction to input if not specified
            try:
                params['direction']
            except:
                params['direction'] = 'input'
                
            if params['type'].lower() == 'Ping'.lower():
                sensor = Ping(self.device, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'].lower() == 'GP2D12'.lower():
                sensor = GP2D12(self.device, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'].lower() == 'Digital'.lower():
                sensor = DigitalSensor(self.device, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'].lower() == 'Analog'.lower():
                sensor = AnalogSensor(self.device, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'].lower() == 'PololuMotorCurrent'.lower():
                sensor = PololuMotorCurrent(self.device, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'].lower() == 'PhidgetsVoltage'.lower():
                sensor = PhidgetsVoltage(self.device, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'].lower() == 'PhidgetsCurrent'.lower():
                sensor = PhidgetsCurrent(self.device, name, params['pin'], params['rate'], self.base_frame)
                
#                if params['type'].lower() == 'MaxEZ1'.lower():
#                    self.sensors[len(self.sensors)]['trigger_pin'] = params['trigger_pin']
#                    self.sensors[len(self.sensors)]['output_pin'] = params['output_pin']

            self.mySensors.append(sensor)
            rospy.loginfo(name + " " + str(params) + " published on topic " + rospy.get_name() + "/sensor/" + name)
            
        # Initialize any joints (servos)
        self.device.joints = dict()
        
        # Read in the joints (if any)    
        joint_params = rospy.get_param("~joints", dict())
        
        if len(joint_params) != 0:
            self.have_joints = True
            
            # Configure each servo
            for name, params in joint_params.iteritems():
                self.device.joints[name] = Servo(self.device, name)

                # Display the joint setup on the terminal
                rospy.loginfo(name + " " + str(params))
            
            # The servo controller determines when to read and write position values to the servos
            self.servo_controller = ServoController(self.device, "ServoController")
            
            # The joint state publisher publishes the latest joint values on the /joint_states topic
            self.joint_state_publisher = JointStatePublisher()
            
#             # Initialize any trajectory action follow controllers
#             controllers = rospy.get_param("~controllers", dict())
#              
#             self.device.controllers = list()
#              
#             for name, params in controllers.items():
#                 try:
#                     controller = controller_types[params["type"]](self.device, name)
#                     self.device.controllers.append(controller)
#                 except Exception as e:
#                     if type(e) == KeyError:
#                         rospy.logerr("Unrecognized controller: " + params["type"])
#                     else:  
#                         rospy.logerr(str(type(e)) + str(e))
#      
#             for controller in self.device.controllers:
#                 controller.startup()

        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.device, self.base_frame)
            
        print "==> ROS Arduino Bridge ready for action!"
    
        # Start polling the sensors, base controller, and servo controller
        while not rospy.is_shutdown():
            for sensor in self.mySensors:
                mutex.acquire()
                sensor.poll()
                mutex.release()
                    
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                mutex.release()
                
            if self.have_joints:
                mutex.acquire()
                self.servo_controller.poll()
                self.joint_state_publisher.poll(self.device.joints.values())
                mutex.release()
            
            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()
            
            if now > self.t_next_sensors:
                msg = SensorState()
                msg.header.frame_id = self.base_frame
                msg.header.stamp = now
                for i in range(len(self.mySensors)):
                    msg.name.append(self.mySensors[i].name)
                    msg.value.append(self.mySensors[i].value)
                try:
                    self.sensorStatePub.publish(msg)
                except:
                    pass
                
                self.t_next_sensors = now + self.t_delta_sensors
            
            r.sleep()
    
    # Service callback functions
    def ServoWriteHandler(self, req):
        self.device.servo_write(req.id, req.value)
        return ServoWriteResponse()
    
    def SetServoSpeedWriteHandler(self, req):
        index = self.joint.values['pin'].index(req.pin)
        name = self.device.joints.keys[index]
        
        # Convert servo speed in deg/s to a step delay in milliseconds
        step_delay = self.device.joints[name].get_step_delay(req.value)

        # Update the servo speed
        self.device.config_servo(pin, step_delay)
        
        return SetServoSpeedResponse()
    
    def ServoReadHandler(self, req):
        pos = self.device.servo_read(req.id)
        return ServoReadResponse(pos)
    
    def DigitalSetDirectionHandler(self, req):
        self.device.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()
    
    def DigitalWriteHandler(self, req):
        self.device.digital_write(req.pin, req.value)
        return DigitalWriteResponse()
    
    def DigitalReadHandler(self, req):
        value = self.device.digital_read(req.pin)
        return DigitalReadResponse(value)
              
    def AnalogWriteHandler(self, req):
        self.device.analog_write(req.pin, req.value)
        return AnalogWriteResponse()
    
    def AnalogReadHandler(self, req):
        value = self.device.analog_read(req.pin)
        return AnalogReadResponse(value)
        
    def shutdown(self):
        rospy.loginfo("Shutting down Arduino node...")

        # Stop the robot
        if self.use_base_controller:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(2)

        # Detach any servos
        if self.have_joints:
            rospy.loginfo("Detaching servos...")
            for joint in self.device.joints.values():
                self.device.detach_servo(joint.pin)
                rospy.sleep(0.2)
        
if __name__ == '__main__':
    myArduino = ArduinoROS()
