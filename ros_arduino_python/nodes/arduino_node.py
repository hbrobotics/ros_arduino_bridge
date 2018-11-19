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
from ros_arduino_python.diagnostics import DiagnosticsUpdater, DiagnosticsPublisher
from ros_arduino_python.cfg import ROSArduinoBridgeConfig
from ros_arduino_python.base_controller import BaseController
from ros_arduino_python.servo_controller import Servo, ServoController
from ros_arduino_python.follow_controller import FollowController
from ros_arduino_python.joint_state_publisher import JointStatePublisher
import dynamic_reconfigure.server
import dynamic_reconfigure.client
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import os, time, thread
from math import radians
from serial.serialutil import SerialException

controller_types = { "follow_controller" : FollowController }

class ArduinoROS():
    def __init__(self):
        rospy.init_node('arduino', log_level=rospy.INFO)

        # Find the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 30))
        r = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        
        # Are we using the base_controller?
        self.use_base_controller = rospy.get_param("~use_base_controller", False)
        
        # Default error threshold (percent) before getting a diagnostics warning
        self.diagnotics_error_threshold = rospy.get_param("~diagnotics_error_threshold", 10)
        
        # Diagnostics update rate
        self.diagnotics_rate = rospy.get_param("~diagnotics_rate", 1.0)
        
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
        
        # A service to attach a PWM servo to a specified pin
        rospy.Service('~servo_attach', ServoAttach, self.ServoAttachHandler)
        
        # A service to detach a PWM servo to a specified pin
        rospy.Service('~servo_detach', ServoDetach, self.ServoDetachHandler)
        
        # A service to position a PWM servo
        rospy.Service('~servo_write', ServoWrite, self.ServoWriteHandler)
        
        # A service to read the position of a PWM servo
        rospy.Service('~servo_read', ServoRead, self.ServoReadHandler)
        
        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        rospy.Service('~digital_pin_mode', DigitalPinMode, self.DigitalPinModeHandler)
        
        # Obsolete: Use DigitalPinMode instead
        rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)
        
        # A service to turn set the direction of an analog pin (0 = input, 1 = output)
        rospy.Service('~analog_pin_mode', AnalogPinMode, self.AnalogPinModeHandler)
        
        # A service to turn a digital sensor on or off
        rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)
        
        # A service to read the value of a digital sensor
        rospy.Service('~digital_read', DigitalRead, self.DigitalReadHandler) 

        # A service to set pwm values for the pins
        rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)
        
        # A service to read the value of an analog sensor
        rospy.Service('~analog_read', AnalogRead, self.AnalogReadHandler)

        # Initialize the device
        self.device = Arduino(self.port, self.baud, self.timeout, debug=True)

        # Make the connection
        if self.device.connect():
            rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
        else:
            rospy.logerr("No serial connection found.")
            os._exit(0)

        # Initialize the base controller if used
        if self.use_base_controller:
            self.base_controller = BaseController(self.device, self.base_frame, self.name + "_base_controller")
            
            # A service to reset the odometry values to 0
            rospy.Service('~reset_odometry', Empty, self.ResetOdometryHandler)
    
            # A service to update the PID parameters Kp, Kd, Ki, and Ko
            rospy.Service('~update_pid', UpdatePID, self.UpdatePIDHandler)
            
            # Fire up the dynamic_reconfigure server
            dyn_server = dynamic_reconfigure.server.Server(ROSArduinoBridgeConfig, self.dynamic_reconfigure_server_callback, namespace=self.name)

            # Connect to the dynamic_reconfigure client
            dyn_client = dynamic_reconfigure.client.Client (self.name, timeout=5)
     
        # Reserve a thread lock
        mutex = thread.allocate_lock()

        # Initialize any sensors
        self.device.sensors = list()

        # Keep a list of IMUs or gyros used for odometry so they can be reset
        self.imu_for_odom = list()

        # Read in the sensors parameter dictionary
        sensor_params = rospy.get_param("~sensors", dict({}))

        # Initialize individual sensors appropriately
        for name, params in sensor_params.iteritems():
            if params['type'].lower() == 'Ping'.lower():
                sensor = Ping(self.device, name, **params)
            elif params['type'].lower() == 'GP2D12'.lower() or params['type'].lower() == 'GP2Y0A21YK0F'.lower():
                sensor = GP2D12(self.device, name, **params)
            elif params['type'].lower() == 'GP2Y0A02YK0F'.lower():
                sensor = GP2Y0A02YK0F(self.device, name, **params)
            elif params['type'].lower() == 'Digital'.lower():
                sensor = DigitalSensor(self.device, name, **params)
            elif params['type'].lower() == 'Analog'.lower():
                sensor = AnalogSensor(self.device, name, **params)
            elif params['type'].lower() == 'AnalogFloat'.lower():
                sensor = AnalogFloatSensor(self.device, name, **params)
            elif params['type'].lower() == 'PololuMotorCurrent'.lower():
                sensor = PololuMotorCurrent(self.device, name, **params)
            elif params['type'].lower() == 'PhidgetsVoltage'.lower():
                sensor = PhidgetsVoltage(self.device, name, **params)
            elif params['type'].lower() == 'PhidgetsCurrent'.lower():
                sensor = PhidgetsCurrent(self.device, name, **params)
            elif params['type'].lower() == 'IMU'.lower():
                sensor = IMU(self.device, name, **params)
                try:
                    if params['use_for_odom']:
                        self.imu_for_odom.append(sensor)
                except:
                    pass
            elif params['type'].lower() == 'Gyro'.lower():
                try:
                    sensor = Gyro(self.device, name, base_controller=self.base_controller, **params)
                except:
                    sensor = Gyro(self.device, name, **params)

                try:
                    if params['use_for_odom']:
                        self.imu_for_odom.append(sensor)
                except:
                    pass
                
#                if params['type'].lower() == 'MaxEZ1'.lower():
#                    self.sensors[len(self.sensors)]['trigger_pin'] = params['trigger_pin']
#                    self.sensors[len(self.sensors)]['output_pin'] = params['output_pin']

            try:
                self.device.sensors.append(sensor)
            
                if params['rate'] != None and params['rate'] != 0:
                    rospy.loginfo(name + " " + str(params) + " published on topic " + rospy.get_name() + "/sensor/" + name)
                else:
                    if sensor.direction == "input":
                        rospy.loginfo(name + " service ready at " + rospy.get_name() + "/" + name + "/read")
                    else:
                        rospy.loginfo(name + " service ready at " + rospy.get_name() + "/" + name + "/write")
            except:
                rospy.logerr("Sensor type " + str(params['type'] + " not recognized."))

        # If at least one IMU or gyro is used for odometry, set the use_imu_heading flag on the base controller
        if self.use_base_controller and len(self.imu_for_odom) > 0:
            self.base_controller.use_imu_heading = True
        
        # Read in the joints (if any)    
        joint_params = rospy.get_param("~joints", dict())
        
        if len(joint_params) != 0:
            self.have_joints = True
            
            self.device.joints = dict()
            
            self.device.joint_update_rate = float(rospy.get_param("~joint_update_rate", 10))
            
            # Configure each servo
            for name, params in joint_params.iteritems():
                try:
                    if params['continuous'] == True:
                        self.device.joints[name] = ContinuousServo(self.device, name)
                    else:
                        self.device.joints[name] = Servo(self.device, name)
                except:
                    self.device.joints[name] = Servo(self.device, name)

                # Display the joint setup on the terminal
                rospy.loginfo(name + " " + str(params))
            
            # The servo controller determines when to read and write position values to the servos
            self.servo_controller = ServoController(self.device, "ServoController")
            
            # The joint state publisher publishes the latest joint values on the /joint_states topic
            self.joint_state_publisher = JointStatePublisher()
            
        # Create the diagnostics updater for the Arduino device
        self.device.diagnostics = DiagnosticsUpdater(self, self.name, self.diagnotics_error_threshold, self.diagnotics_rate, create_watchdog=True)
        
        # Create the overall diagnostics publisher
        self.diagnostics_publisher = DiagnosticsPublisher(self)
            
        # Initialize any trajectory action follow controllers
        controllers = rospy.get_param("~controllers", dict())
          
        self.device.controllers = list()
          
        for name, params in controllers.items():
            try:
                controller = controller_types[params["type"]](self.device, name)
                self.device.controllers.append(controller)
            except Exception as e:
                if type(e) == KeyError:
                    rospy.logerr("Unrecognized controller: " + params["type"])
                else:  
                    rospy.logerr(str(type(e)) + str(e))
  
        for controller in self.device.controllers:
            controller.startup()
            
        print "\n==> ROS Arduino Bridge ready for action!"
        
        last_sonar_update = rospy.Time.now()
    
        # Start polling the sensors, base controller, and servo controller
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            
            # Heartbeat/watchdog test for the serial connection
            try:
                # Update read counters
                self.device.diagnostics.reads += 1
                self.device.diagnostics.total_reads += 1
                self.device.serial_port.inWaiting()
                # Add this heartbeat to the frequency status diagnostic task
                self.device.diagnostics.freq_diag.tick()
                # Let the diagnostics updater know we're still alive
                self.device.diagnostics.watchdog = True
            except IOError:
                # Update error counter
                self.device.diagnostics.errors += 1
                rospy.loginfo("Lost serial connection. Waiting to reconnect...")
                # Let the diagnostics updater know that we're down
                self.device.diagnostics.watchdog = False
                self.device.close()
                with mutex:
                    while True:
                        try:
                            self.device.open()
                            while True:
                                self.device.serial_port.write('\r')
                                test = self.device.serial_port.readline().strip('\n').strip('\r')
                                rospy.loginfo("Waking up serial port...")
                                if test == 'Invalid Command':
                                    self.device.serial_port.flushInput()
                                    self.device.serial_port.flushOutput()
                                    break
                            rospy.loginfo("Serial connection re-established.")
                            break
                        except:
                            r.sleep()
                            self.diagnostics_publisher.update()
                            continue
            
            # Poll any sensors
            for sensor in self.device.sensors:
                if sensor.rate != 0:
                    with mutex:
                        sensor.poll()
                                     
            # Poll the base controller
            if self.use_base_controller:
                with mutex:
                    self.base_controller.poll()
                
            # Poll any joints
            if self.have_joints:
                with mutex:
                    self.servo_controller.poll()
                    self.joint_state_publisher.poll(self.device.joints.values())
                                
            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()
            
            if now > self.t_next_sensors:
                msg = SensorState()
                msg.header.frame_id = self.base_frame
                msg.header.stamp = now
                for i in range(len(self.device.sensors)):
                    if self.device.sensors[i].message_type != MessageType.IMU:
                        msg.name.append(self.device.sensors[i].name)
                        msg.value.append(self.device.sensors[i].value)
                try:
                    self.sensorStatePub.publish(msg)
                except:
                    pass
                
                self.t_next_sensors = now + self.t_delta_sensors
                
            # Update diagnostics and publish
            self.diagnostics_publisher.update()
            
            r.sleep()
    
    # Service callback functions
    def ServoAttachHandler(self, req):
        self.device.attach_servo(req.id)
        return ServoAttachResponse()
    
    def ServoDetachHandler(self, req):
        self.device.detach_servo(req.id)
        return ServoDetachResponse()
    
    def ServoWriteHandler(self, req):
        self.device.servo_write(req.id, req.position)
        return ServoWriteResponse()
    
#     def ServoSpeedWriteHandler(self, req):
#         delay = 
#         self.device.servo_delay(req.id, delay)
#         return ServoSpeedResponse()
# 
#         # Convert servo speed in deg/s to a step delay in milliseconds
#         step_delay = self.device.joints[name].get_step_delay(req.value)
# 
#         # Update the servo speed
#         self.device.config_servo(pin, step_delay)
#         
#         return SetServoSpeedResponse()
    
    def ServoReadHandler(self, req):
        position = self.device.servo_read(req.id)
        return ServoReadResponse(position)
    
    def DigitalPinModeHandler(self, req):
        self.device.digital_pin_mode(req.pin, req.direction)
        return DigitalPinModeResponse()
    
    # Obsolete: use DigitalPinMode instead
    def DigitalSetDirectionHandler(self, req):
        self.device.digital_pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()
    
    def DigitalWriteHandler(self, req):
        self.device.digital_write(req.pin, req.value)
        return DigitalWriteResponse()
    
    def DigitalReadHandler(self, req):
        value = self.device.digital_read(req.pin)
        return DigitalReadResponse(value)
    
    def AnalogPinModeHandler(self, req):
        self.device.analog_pin_mode(req.pin, req.direction)
        return AnalogPinModeResponse()
              
    def AnalogWriteHandler(self, req):
        self.device.analog_write(req.pin, req.value)
        return AnalogWriteResponse()
    
    def AnalogReadHandler(self, req):
        value = self.device.analog_read(req.pin)
        return AnalogReadResponse(value)

    def ResetOdometryHandler(self, req):
        if self.use_base_controller:
            self.base_controller.reset_odometry()
            for imu in self.imu_for_odom:
                imu.reset()
        else:
            rospy.logwarn("Not using base controller!")
        return EmptyResponse()

    def UpdatePIDHandler(self, req):
        if self.use_base_controller:
           self.device.update_pid(req.Kp, req.Kd, req.Ki, req.Ko)
           rospy.loginfo("Updating PID parameters: Kp=%0.3f, Kd=%0.3f, Ki=%0.3f, Ko=%0.3f" %(req.Kp, req.Kd, req.Ki, req.Ko))
        else:
            rospy.logwarn("Not using base controller!")
        return UpdatePIDResponse()

    def dynamic_reconfigure_server_callback(self, config, level):
        if self.use_base_controller:
            try:
                if self.base_controller.Kp != config['Kp']:
                    self.base_controller.Kp = config['Kp']

                if self.base_controller.Kd != config['Kd']:
                    self.base_controller.Kd = config['Kd']

                if self.base_controller.Ki != config['Ki']:
                    self.base_controller.Ki = config['Ki']

                if self.base_controller.Ko != config['Ko']:
                    self.base_controller.Ko = config['Ko']

                if self.base_controller.accel_limit != config['accel_limit']:
                    self.base_controller.accel_limit = config['accel_limit']
                    self.base_controller.max_accel = self.base_controller.accel_limit * self.base_controller.ticks_per_meter / self.base_controller.rate

                if self.base_controller.odom_linear_scale_correction != config['odom_linear_scale_correction']:
                    self.base_controller.odom_linear_scale_correction = config['odom_linear_scale_correction']
                    
                if self.base_controller.odom_angular_scale_correction != config['odom_angular_scale_correction']:
                    self.base_controller.odom_angular_scale_correction = config['odom_angular_scale_correction']

                self.device.update_pid(self.base_controller.Kp, self.base_controller.Kd, self.base_controller.Ki, self.base_controller.Ko)

                rospy.loginfo("Updating PID parameters: Kp=%0.2f, Kd=%0.2f, Ki=%0.2f, Ko=%0.2f, accel_limit=%0.2f" %(self.base_controller.Kp, self.base_controller.Kd, self.base_controller.Ki, self.base_controller.Ko, self.base_controller.accel_limit))
            except Exception as e:
                print e

        return config
        
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
                
        # Close the serial port
        self.device.close()
        
if __name__ == '__main__':
    try:
        myArduino = ArduinoROS()
    except KeyboardInterrupt:
        try:
            myArduino.device.serial_port.close()
        except:
            rospy.logerr("Serial exception trying to close port.")
            os._exit(0)
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)
        
