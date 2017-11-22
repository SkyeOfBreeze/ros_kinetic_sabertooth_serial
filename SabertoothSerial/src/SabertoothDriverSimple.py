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

import rospy, serial, Queue
from std_msgs.msg import String
from SabertoothSerial.msg import SabertoothMotor
from geometry_msgs.msg import Twist
import tf.transformations


class SerialMotorControl:
    serialPort = '/dev/ttyUSB0'
    timeout = 0
    # Setup usb serial communication. If you have multiple usb serial devices, this may need to be changed.
    # This cannot detect which one is the SaberTooth
    ard = 0
    publishEnabled = False
    pub = 0
    queue = Queue.Queue()
    x_min = 1
    x_max = 1
    r_min = 1
    r_max = 1
    x = 0
    y = 0

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def send_command(self, x, y):
        if self.publishEnabled:
            self.publish_raw(x, y)
        else:
            x = -x  # invert x
            V = (1 - abs(x)) * (y / 1) + y  # Calculate R+L
            W = (1 - abs(y)) * (x / 1) + x  # Calculate R-L
            R = (V + W) / 2  # Calculate R
            L = (V - W) / 2  # Calculate L
            self.motor_raw_process(0, L)
            self.motor_raw_process(1, R)

    # Internally set ROS publisher and use it. calling rosrun on this file will make it a subscriber for motors
    def set_publish_event(self, publish):
        self.publishEnabled = publish
        if publish:
            rospy.init_node('cmd_vel_motor_tester', anonymous=True)
            self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        else:
            self.ard = serial.Serial(self.serialPort, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

    def set_serial_port(self, port):
        self.serialPort = port
        if not self.publishEnabled:
            self.ard = serial.Serial(self.serialPort, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

    def publish_raw(self, y, x):
        twist = Twist()
        twist.linear.x = (1 - y) * (self.x_max - self.x_min) + self.x_min
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = (1 - x) * (self.r_max - self.r_min) + self.r_min

        if twist.linear.x > self.x_max:
            twist.linear.x = self.x_max
        if twist.linear.x < self.x_min:
            twist.linear.x = self.x_min
        if twist.angular.z > self.r_max:
            twist.angular.z = self.r_max
        if twist.angular.z < self.r_min:
            twist.angular.z = self.r_min
        self.pub.publish(twist)

    def get_byte_of_motor(self, motor, power):
        power = self.constrain(power, -127, 127);
        magnitude = abs(power) >> 1;
        command = 0
        if motor == 0:
            if power < 0:
                command = 63 - magnitude
            else:
                command = 64 + magnitude
        else:
            if motor == 1:
                if power < 0:
                    command = 191 - magnitude
                else:
                    command = 192 + magnitude
        command = self.constrain(command, 1, 254);
        return command

    def motor_raw_process(self, motor, power):
        data = self.get_byte_of_motor(motor, power)
        self.motor_raw(data)

    def motor_raw(self, data):
        self.ard.write(chr(data))

    def drive_both(self, x, y):
        # idk how to convert to x and y
        self.send_command(x, y)

    def drive_both_raw(self, left, right):
        self.motor_raw_process(0, left)
        self.motor_raw_process(1, right)

    def drive(self, power):
        self.drive_both(power, power)

    def drive_forward(self, power):
        self.drive(power)

    def drive_backward(self, power):
        self.drive(-power)

    def drive_left(self, power):
        self.drive_both(-power, power)

    def drive_right(self, power):
        self.drive_both(power, -power)

    def stop(self):
        self.drive(0)

    def twist(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands

        # cmd_vel.angle is the angular z component and cmd_vel.speed the linear x component
        # https://answers.ros.org/question/209963/cmd_veltwist-transform-twist-message-into-left-and-right-motor-commands/?answer=209966#post-id-209966
        wheel_dist = 50.8  # cm
        speed_wish_right = (msg.angular.z * wheel_dist) / 2 + msg.linear.x
        speed_wish_left = msg.linear.x * 2 - speed_wish_right

        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        self.drive_both_raw(speed_wish_left, speed_wish_right)

    def listener(self):
        # don't let this run unless it is a node. Better Idea; don't allow this to be a node
        if __name__ == '__main__':
            # In ROS, nodes are uniquely named. If two nodes with the same
            # name are launched, the previous one is kicked off. The
            # anonymous=True flag means that rospy will choose a unique
            # name for our 'listener' node so that multiple listeners can
            # run simultaneously.
            # rospy.init_node('sabertooth_driver', anonymous=True)
            rospy.init_node('cmd_vel_drive_train_listener')
            rospy.Subscriber("/cmd_vel", Twist, self.twist)
            # rospy.Subscriber('motor_control_drive', SabertoothMotor, self.callback)
            # rospy.Subscriber('motor_control_drive_raw', String, self.raw_callback)
            time_since_last_message = rospy.get_time()
            while not rospy.is_shutdown():
                while not self.queue.empty():
                    time_since_last_message = rospy.get_time()
                    cmd_vel = self.queue.get()
                    self.twist(cmd_vel)
                else:
                    self.timeout = self.timeout + 1
                    if rospy.get_time() - time_since_last_message > 1:
                        self.stop()
            # spin() simply keeps python from exiting until this node is stopped
            rospy.spin()

    def update(self):
        self.drive_both(self.x, self.y)


if __name__ == '__main__':
    SerialControl = SerialMotorControl()
    SerialControl.set_publish_event(False)
    try:
        SerialControl.listener()
    except rospy.ROSInterruptException:
        pass
    SerialControl.stop()
