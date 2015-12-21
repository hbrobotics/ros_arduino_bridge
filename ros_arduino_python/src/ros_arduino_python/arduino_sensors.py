#!/usr/bin/env python

"""
    Sensor class for the arudino_python package
    
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
from sensor_msgs.msg import Range
from ros_arduino_msgs.msg import *
from ros_arduino_msgs.srv import *
from math import pow

LOW = 0
HIGH = 1

INPUT = 0
OUTPUT = 1

class MessageType:
    ANALOG = 0
    DIGITAL = 1
    RANGE = 2
    FLOAT = 3
    INT = 4
    BOOL = 5
    
class Sensor(object):
    def __init__(self, device, name, pin=None, rate=0, direction="input", frame_id="base_link", **kwargs):
        self.device = device
        self.name = name
        self.pin = pin
        self.rate = rate
        self.direction = direction
        self.frame_id = frame_id

        self.value = None
        
        if self.rate != 0:
            self.t_delta = rospy.Duration(1.0 / self.rate)
            self.t_next = rospy.Time.now() + self.t_delta
    
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            if self.direction == "input":
                try:
                    self.value = self.read_value()
                except:
                    return
            else:
                try:
                    self.ack = self.write_value()
                except:
                    return          
    
            # For range sensors, assign the value to the range message field
            if self.message_type == MessageType.RANGE:
                self.msg.range = self.value
            else:
                self.msg.value = self.value

            # Add a timestamp and publish the message
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            
            self.t_next = now + self.t_delta
    
class AnalogSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = Analog()
        self.msg.header.frame_id = self.frame_id
        
        # Create the publisher
        self.pub = rospy.Publisher("~sensor/" + self.name, Analog, queue_size=5)

        if self.direction == "output":
            self.device.pin_mode(self.pin, OUTPUT)
            # Create the write service
            rospy.Service('~' + self.name + '/write', AnalogSensorWrite, self.sensor_write_handler)
        else:
            self.device.pin_mode(self.pin, INPUT)
            # Create the read service
            rospy.Service('~' + self.name + '/read', AnalogSensorRead, self.sensor_read_handler)

        self.value = LOW
        
    def read_value(self):
        return self.device.analog_read(self.pin)
    
    def write_value(self, value):
        return self.device.analog_write(self.pin, value)

    def sensor_read_handler(self, req=None):
        value = self.read_value()
        return AnalogSensorReadResponse(value)
    
    def sensor_write_handler(self, req):
        self.write_value(req.value)
        return AnalogSensorWriteResponse()

class AnalogFloatSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogFloatSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = AnalogFloat()
        self.msg.header.frame_id = self.frame_id
        
        # Create the publisher
        self.pub = rospy.Publisher("~sensor/" + self.name, AnalogFloat, queue_size=5)

        if self.direction == "output":
            self.device.pin_mode(self.pin, OUTPUT)
            # Create the write service
            rospy.Service('~' + self.name + '/write', AnalogFloatSensorWrite, self.sensor_write_handler)
        else:
            self.device.pin_mode(self.pin, INPUT)
            # Create the read service
            rospy.Service('~' + self.name + '/read', AnalogFloatSensorRead, self.sensor_read_handler)

        self.value = LOW
        
    def read_value(self):
        return self.device.analog_read(self.pin)
    
    def write_value(self, value):
        return self.device.analog_write(self.pin, value)
    
    def sensor_read_handler(self, req=None):
        value = self.read_value()
        return AnalogFloatSensorReadResponse(value)
    
    def sensor_write_handler(self, req):
        self.write_value(req.value)
        return AnalogFloatSensorWriteResponse()
        
class DigitalSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(DigitalSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.BOOL
        
        self.msg = Digital()
        self.msg.header.frame_id = self.frame_id

        # Create the publisher        
        self.pub = rospy.Publisher("~sensor/" + self.name, Digital, queue_size=5)

        if self.direction == "output":
            self.device.pin_mode(self.pin, OUTPUT)
            # Create the write service
            rospy.Service('~' + self.name + '/write', DigitalSensorWrite, self.sensor_write_handler)
        else:
            self.device.pin_mode(self.pin, INPUT)
            # Create the read service
            rospy.Service('~' + self.name + '/read', DigitalSensorRead, self.sensor_read_handler)

        self.value = LOW
        
    def read_value(self):
        return self.device.digital_read(self.pin)
    
    def write_value(self):
        # Alternate HIGH/LOW when writing at a fixed rate
        self.value = not self.value
        return self.device.digital_write(self.pin, self.value)
    
    def sensor_read_handler(self, req=None):
        value = self.read_value()
        return DigitalSensorReadResponse(value)
    
    def sensor_write_handler(self, req):
        self.write_value(req.value)
        return DigitalSensorWriteResponse()  
    
class RangeSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(RangeSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.RANGE
        
        self.msg = Range()
        self.msg.header.frame_id = self.frame_id

        # Create the publisher        
        self.pub = rospy.Publisher("~sensor/" + self.name, Range, queue_size=5)
        
    def read_value(self):
        self.msg.header.stamp = rospy.Time.now()

class SonarSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(SonarSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.ULTRASOUND
        
class IRSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(IRSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.INFRARED
        
class Ping(SonarSensor):
    def __init__(self,*args, **kwargs):
        super(Ping, self).__init__(*args, **kwargs)
                
        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        # The Arduino Ping code returns the distance in centimeters
        cm = self.device.ping(self.pin)
        
        # Convert it to meters for ROS
        distance = cm / 100.0
        
        return distance
        
class GP2D12(IRSensor):
    def __init__(self, *args, **kwargs):
        super(GP2D12, self).__init__(*args, **kwargs)
        
        self.msg.field_of_view = 0.001
        self.msg.min_range = 0.10
        self.msg.max_range = 0.80
        
    def read_value(self):
        value = self.device.analog_read(self.pin)
        
        # The GP2D12 cannot provide a meaning result closer than 3 cm.
        if value <= 3.0:
            return float('NaN')
        
        try:
            distance = pow(4187.8 / value, 1.106)
            #distance = (6787.0 / (float(value) - 3.0)) - 4.0
        except:
            return float('NaN')
            
        # Convert to meters
        distance /= 100.0
        
        # If we get a spurious reading, set it to the max_range
        if distance > self.msg.max_range: distance = float('NaN')
        if distance < self.msg.min_range: distance = float('NaN')
        
        return distance
    
class PololuMotorCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PololuMotorCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Pololu source code
        milliamps = self.device.analog_read(self.pin) * 34
        return milliamps / 1000.0
    
class PhidgetsVoltage(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsVoltage, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation
        voltage = 0.06 * (self.device.analog_read(self.pin) - 500.)
        return voltage
    
class PhidgetsCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation for the 20 amp DC sensor
        current = 0.05 * (self.device.analog_read(self.pin) - 500.)
        return current
    
class MaxEZ1Sensor(SonarSensor):
    def __init__(self, *args, **kwargs):
        super(MaxEZ1Sensor, self).__init__(*args, **kwargs)
        
        self.trigger_pin = kwargs['trigger_pin']
        self.output_pin = kwargs['output_pin']
        
        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        return self.device.get_MaxEZ1(self.trigger_pin, self.output_pin)

            
            
