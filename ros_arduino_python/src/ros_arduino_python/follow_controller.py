#!/usr/bin/env python

"""
  follow_controller.py - controller for a kinematic chain
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy, actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from controllers import *

class FollowController(Controller):
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self, device, name):
        Controller.__init__(self, device, name)
        # Parameters: rates and joints
        self.rate = rospy.get_param('~controllers/' + name + '/rate', 50.0)
        self.joints = rospy.get_param('~controllers/' + name + '/joints')
        self.index = rospy.get_param('~controllers/'+ name + '/index', len(device.controllers))
        for joint in self.joints:
            self.device.joints[joint].controller = self

        # Action server
        name = rospy.get_param('~controllers/' + name + '/action_name', 'follow_joint_trajectory')
        self.server = actionlib.SimpleActionServer(name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)

        # Good old trajectory
        rospy.Subscriber(self.name + '/command', JointTrajectory, self.command_cb)
        self.executing = False

        rospy.loginfo("Started FollowController ("+self.name+"). Joints: " + str(self.joints) + " on C" + str(self.index))

    def startup(self):
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory
        
        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
                    msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
                    rospy.logerr(msg)
                    self.server.set_aborted(text=msg)
                    return
            rospy.logwarn("Extra joints in trajectory")

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if self.execute_trajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        rospy.loginfo(self.name + ": Done.")
    
    def command_cb(self, msg):
        # Don't execute if executing an action
        if self.server.is_active():
            rospy.loginfo(self.name + ": Received trajectory, but action is active")
            return
        self.executing = True
        self.execute_trajectory(msg)
        self.executing = False    

    def execute_trajectory(self, traj):
        rospy.loginfo("Executing trajectory")
        rospy.logdebug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            rospy.logerr("Invalid joint in trajectory.")
            return False

        # Get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = rospy.Time.now()
        
        # Set the start time for each joint
        for i in range(len(self.joints)):
            self.device.joints[self.joints[i]].time_move_started = start

        r = rospy.Rate(self.rate)
        last = [ self.device.joints[joint].position for joint in self.joints ]
        for point in traj.points:
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
            while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                err = [ (d-c) for d,c in zip(desired,last) ]
                velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]
                rospy.logdebug(err)
                for i in range(len(self.joints)):
                    if err[i] > 0.001 or err[i] < -0.001:
                        cmd = err[i] 
                        top = velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        last[i] += cmd
                        self.device.joints[self.joints[i]].time_move_started = rospy.Time.now()
                        self.device.joints[self.joints[i]].in_trajectory = True
                        self.device.joints[self.joints[i]].is_moving = True
                        self.device.joints[self.joints[i]].desired = last[i]
                    else:
                        self.device.joints[self.joints[i]].speed = 0.0
                        self.device.joints[self.joints[i]].in_trajectory = True
                        self.device.joints[self.joints[i]].is_moving = False
                        velocity[i] = 0
                r.sleep()
        return True

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg
