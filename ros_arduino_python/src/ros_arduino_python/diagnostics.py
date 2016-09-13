#!/usr/bin/env python

"""
    Diagnostics classes for the arudino_python package
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

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
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

class DiagnosticsPublisher():
    """ Class to handle publications of diagnostics messages. """

    def __init__(self, arduino):
        self.arduino = arduino
        self.device = self.arduino.device

    def update(self):
        # First the arduino itself
        self.device.diagnostics.diag_updater.update()
     
        # Then the sensors
        for sensor in self.device.sensors:
            sensor.diagnostics.diag_updater.update()
                    
        # Next the servos
        if self.arduino.have_joints:
            for servo in self.arduino.servo_controller.servos:
                servo.diagnostics.diag_updater.update()
                    
        # Finally, the base controller
        if self.arduino.use_base_controller:
            self.arduino.base_controller.diagnostics.diag_updater.update()

class DiagnosticsUpdater():
    """ Class to return diagnostic status from a component. """
    
    def __init__(self, component, name, error_threshold=10, rate=1.0, create_watchdog=False):
        self.component = component
        
        self.name = name
            
        # Determines the OK, WARN and ERROR status flags
        self.error_threshold = error_threshold
        
        # Create a diagnostics updater
        self.diag_updater = diagnostic_updater.Updater()
        
        # Set the period from the rate
        self.diag_updater.period = 1.0 / rate
        
        # Set the hardware ID to name + pin
        try:
            self.diag_updater.setHardwareID(self.name + '_pin_' + str(self.component.pin))
        except:
            self.diag_updater.setHardwareID(self.name)
            
        # Create a frequency monitor that tracks the publishing frequency for this component
        if self.component.rate > 0:
            if "Servo" in str(self.component):
                freq_bounds = diagnostic_updater.FrequencyStatusParam({'min': self.component.device.joint_update_rate, 'max': self.component.device.joint_update_rate}, 0.3, 5)
                self.freq_diag = diagnostic_updater.HeaderlessTopicDiagnostic(self.component.name + '_freq', self.diag_updater, freq_bounds)
            else:
                 freq_bounds = diagnostic_updater.FrequencyStatusParam({'min': self.component.rate, 'max': self.component.rate}, 0.3, 5)
                 self.freq_diag = diagnostic_updater.HeaderlessTopicDiagnostic(self.component.name + '_freq', self.diag_updater, freq_bounds)

        # Create an error monitor that tracks timeouts, serial exceptions etc
        self.error_diag = diagnostic_updater.FunctionDiagnosticTask(self.name, self.get_error_rate)
        self.diag_updater.add(self.error_diag)
        
        # Create a watchdog diagnostic to monitor the connection status
        if create_watchdog:
            self.watchdog_diag = diagnostic_updater.FunctionDiagnosticTask(self.name, self.get_connection_status)
            self.diag_updater.add(self.watchdog_diag)
            
        # Counters used for diagnostics
        self.reads = 0
        self.total_reads = 0
        self.errors = 0
        self.error_rates = [0.0]
        
        # Connection status for device
        self.watchdog = False
        
    def get_error_rate(self, stat):        
        error_rate = 0.0

        # Wait until we have some data before computing error rates
        if self.reads + self.errors > 100:
            error_rate = (self.errors * 100.0) / (self.reads + self.errors)
            self.error_rates.append(error_rate)
            # Keep only the past 1000 measurements (100 x 10) so that error rates are current
            if len(self.error_rates) > 10:
                self.error_rates = self.error_rates[-10:]
            self.reads = 0
            self.errors = 0
            
        error_rate = sum(self.error_rates) / len(self.error_rates)
            
        if error_rate > 3 * self.error_threshold:
            stat.summary(DiagnosticStatus.ERROR, "Excessive Error Rate")
            
        elif error_rate > 2 * self.error_threshold:
            stat.summary(DiagnosticStatus.WARN, "Moderate Error Rate")
            
        else:
            stat.summary(DiagnosticStatus.OK, "Error Rate OK")
            
        stat.add("Reads", self.total_reads)
        stat.add("Error Rate", error_rate)

        return stat
    
    def get_connection_status(self, stat):
        stat.path = self.name + "/Watchdog"
        if not self.watchdog:
            stat.summary(DiagnosticStatus.ERROR, "Connnection lost!")
        else:
            stat.summary(DiagnosticStatus.OK, "Alive")

        stat.add("Watchdog", self.watchdog)
        return stat

            