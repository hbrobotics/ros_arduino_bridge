#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
    
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
import thread
from math import pi as PI
import os, time, sys, traceback
from serial.serialutil import SerialException, SerialTimeoutException
import serial
from exceptions import Exception

# Possible errors when reading/writing to the Arduino
class CommandErrorCode:
    SUCCESS = 0
    TIMEOUT = 1
    NOVALUE = 2
    SERIALEXCEPTION = 3

    ErrorCodeStrings = ['SUCCESS', 'TIMEOUT', 'NOVALUE', 'SERIALEXCEPTION']

# An exception class to handle these errors
class CommandException(Exception):
    def __init__(self, code):
        self.code = code
    def __str__(self):
        return repr(self.code)

class Arduino:
    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5, debug=False):
        
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.debug = debug

        # Keep things thread safe
        self.mutex = thread.allocate_lock()

    def connect(self):
        try:
            rospy.loginfo("Looking for the Arduino on port " + str(self.port) +  " ...")
            
            # The port has to be open once with the default baud rate before opening again for real
            self.serial_port = serial.Serial(port=self.port)
          
            # Needed for Leonardo only
            while not self.serial_port.isOpen():
                time.sleep(self.timeout)

            # Now open the port with the real settings.  An initial timeout of at least 1.0 seconds seems to work best
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=max(1.0, self.timeout))

            # It can take time for the serial port to wake up
            max_attempts = 10
            attempts = 0
            timeout = self.timeout

            self.serial_port.write('\r')
            time.sleep(timeout)
            test = self.serial_port.read()

            #  Wake up the serial port
            while attempts < max_attempts and test == '':
                attempts += 1
                self.serial_port.write('\r')
                time.sleep(timeout)
                test = self.serial_port.read()
                rospy.loginfo("Waking up serial port attempt " + str(attempts) + " of " + str(max_attempts))

            if test == '':
                raise SerialException

            # Reset the timeout to the user specified timeout
            self.serial_port.timeout = self.timeout
            self.serial_port.writeTimeout = self.timeout

            # Test the connection by reading the baudrate
            attempts = 0
            while self.get_baud() != self.baudrate and attempts < max_attempts:
                attempts += 1
                self.serial_port.flushInput()
                self.serial_port.flushOutput()
                rospy.loginfo("Connecting...")
                time.sleep(timeout)
            try:
                self.serial_port.inWaiting()
                rospy.loginfo("Connected at " + str(self.baudrate))
                rospy.loginfo("Arduino is ready!")
            except IOError:
                raise SerialException

        except SerialException:
            rospy.logerr("Serial Exception:")
            rospy.logerr(sys.exc_info())
            rospy.logerr("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            rospy.logerr("Cannot connect to Arduino!  Make sure it is plugged in to your computer.")
            return False

        return True

    def open(self): 
        ''' Open the serial port.
        '''
        self.serial_port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.serial_port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.serial_port.write(cmd + '\r')

    def execute(self, cmd, timeout=0.5):
        ''' Thread safe execution of "cmd" on the Arduino returning a single value.
        '''
        self.mutex.acquire()

        value = None
        error = None

        try:
            start = time.time()
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            self.serial_port.write(cmd + '\r')
            value = self.serial_port.readline().strip('\n').strip('\r')
        except SerialException:
            self.print_debug_msg("Command " + str(cmd) + " failed with Serial Exception")
            error = CommandErrorCode.SERIALEXCEPTION
        except SerialTimeoutException:
            self.print_debug_msg("Command " + str(cmd) + " timed out")
            error = CommandErrorCode.TIMEOUT

        duration = time.time() - start

        if error is None and (value is None or len(value) == 0):
            duration = time.time() - start
            if duration > timeout:
                self.print_debug_msg("Command " + str(cmd) + " timed out")
                error = CommandErrorCode.TIMEOUT
            else:
                self.print_debug_msg("Command " + str(cmd) + " did not return a value")
                error = CommandErrorCode.NOVALUE

        self.mutex.release()

        if error:
            raise CommandException(error)

        return value

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        try:
            values = self.execute(cmd).split()
        except CommandException as e:
            values = None

        return values

    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        try:
            ack = self.execute(cmd)
            return ack == "OK"
        except CommandException as e:
            return False

    def print_debug_msg(self, msg):
        if self.debug:
            rospy.logwarn(msg)
    
    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        return self.execute_ack(cmd)

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        try:
            return int(self.execute('b'))
        except:
            return None

    def get_encoder_counts(self):
        ''' Read the current encoder counts
        '''
        values = self.execute_array('e')

        if len(values) != 2:
            return None
        else:
            return map(int, values)

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')

    def get_imu_data(self):
        '''
        IMU data is assumed to be returned in the following order:
    
        [ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, ch]
    
        where a stands for accelerometer, g for gyroscope and m for magnetometer.
        The last value ch stands for "compensated heading" that some IMU's can 
        compute to compensate magnetic heading from the current roll and pitch.
        '''
        values = self.execute_array('i')

        if len(values) != 12:
            return None
        else:
            return map(float, values)

    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        return self.execute_ack('m %d %d' %(right, left))
    
    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)
        
    def analog_pin_mode(self, pin, mode):
        return self.execute_ack('c A%d %d' %(pin, mode))
            
    def analog_read(self, pin):
        try:
            return int(self.execute('a %d' %pin))
        except:
            return None
    
    def analog_write(self, pin, value):
        return self.execute_ack('x %d %d' %(pin, value))
    
    def digital_read(self, pin):
        try:
            return int(self.execute('d %d' %pin))
        except:
            return None
    
    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))
    
    def digital_pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))
    
    def config_servo(self, pin, step_delay):
        ''' Configure a PWM servo '''
        return self.execute_ack('j %d %u' %(pin, step_delay))

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in degrees from 0-180
        '''
        return self.execute_ack('s %d %d' %(id, pos))
    
    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is in degrees
        '''
        return int(self.execute('t %d' %id))
    
    def set_servo_delay(self, id, delay):
        ''' Usage: set_servo_delay(id, delay)
            Set the delay in ms inbetween servo position updates.  Controls speed of servo movement.
        '''
        return self.execute_ack('v %d %d' %(id, delay))

    def detach_servo(self, id):
        ''' Usage: detach_servo(id)
            Detach a servo from control by the Arduino
        '''        
        return self.execute_ack('z %d' %id)
    
    def attach_servo(self, id):
        ''' Usage: attach_servo(id)
            Attach a servo to the Arduino
        '''        
        return self.execute_ack('y %d' %id)
    
    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,q
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return int(self.execute('p %d' %pin));
    
#    def get_maxez1(self, triggerPin, outputPin):
#        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
#            sensor connected to the General Purpose I/O lines, triggerPin, and
#            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
#            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
#            power up, otherwise it wont range correctly for object less than 6
#            inches away! The sensor reading defaults to use English units
#            (inches). The sensor distance resolution is integer based. Also, the
#            maxsonar trigger pin is RX, and the echo pin is PW.
#        '''
#        return self.execute('z %d %d' %(triggerPin, outputPin)) 
 

""" Basic test for connectivity """
if __name__ == "__main__":
    if os.name == "posix":
        portName = "/dev/ttyACM0"
    else:
        portName = "COM43" # Windows style COM port.
        
    baudRate = 57600

    myArduino = Arduino(port=portName, baudrate=baudRate, timeout=0.5)
    myArduino.connect()
     
    print "Sleeping for 1 second..."
    time.sleep(1)   
    
    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()
    
    print "Connection test successful.",
    
    myArduino.stop()
    myArduino.close()
    
    print "Shutting down Arduino."
    
