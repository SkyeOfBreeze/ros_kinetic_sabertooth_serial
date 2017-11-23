#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017,
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Brendon Telman nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# Simple Serial Motor Driver for the SaberTooth Motor Drivers
# Version 1
import Queue
import rospy
from SabertoothSerial.SabertoothDriverSimple import SerialMotorControl
from geometry_msgs.msg import Twist


class SimpleSerialTwist:
    motors = SerialMotorControl('/dev/ttyUSB0')
    time_since_last_message = 0

    timeout = 0
    # Setup usb serial communication. If you have multiple usb serial devices, this may need to be changed.
    # This cannot detect which one is the SaberTooth
    x_min = 0
    x_max = 1
    r_min = 0
    r_max = 1
    x = 0
    y = 0
    speed_wish_right = 0
    speed_wish_left = 0

    def twist(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))
        self.time_since_last_message = rospy.get_time()
        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands

        # cmd_vel.angle is the angular z component and cmd_vel.speed the linear x component
        # https://answers.ros.org/question/209963/cmd_veltwist-transform-twist-message-into-left-and-right-motor-commands/?answer=209966#post-id-209966
        wheel_dist = 50.8  # cm
        self.speed_wish_right = (msg.angular.z * wheel_dist) / 2 + msg.linear.x
        self.speed_wish_left = msg.linear.x * 2 - self.speed_wish_right
        self.motors.drive_both(int(self.speed_wish_left * 100), int(self.speed_wish_right * 100))

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.loginfo("init_node SimpleSerialTwist")
        rospy.loginfo("Subscriber=cmd_vel")
        rospy.Subscriber("cmd_vel", Twist, self.twist)
        self.time_since_last_message = rospy.get_time()
        while not rospy.is_shutdown():
            if rospy.get_time() - self.time_since_last_message > 1:  # Timeout checking
                self.motors.stop()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('SimpleSerialTwist', anonymous=True)
    rospy.loginfo("start program")
    SerialTwist = SimpleSerialTwist()
    rospy.loginfo("init SimpleSerialTwist")
    try:
        rospy.loginfo("listen")
        SerialTwist.listener()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("end of program")
    SerialTwist.motors.stop()
