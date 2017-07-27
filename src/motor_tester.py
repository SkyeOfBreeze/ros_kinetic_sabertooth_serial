#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Brendon Telman - brendon[at]btelman.org
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

## Simple motor tester demo for the Sabertooth Packetized Serial Mode

import rospy
#import getch
import sys
import select
import termios
import contextlib
from SabertoothDriver.msg import SabertoothMotor
from SabertoothDriverSimple import SerialMotorControl
from time import sleep

motors = SerialMotorControl()

def init():
    if __name__ == '__main__':
        promptPublish()
        motors.stop()
        try:
            looper()
        except rospy.ROSInterruptException:
            pass
        motors.stop()

def promptPublish():
    key = raw_input('Publish to driver instead of direct access? Y/n')
    if key == 'Y':
        motors.setPublishEvent(True)
    else:
        motors.setPublishEvent(False)
    
        
def getch():
    with raw_mode(sys.stdin):
        try:
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = ord(sys.stdin.read(1))
                # key = sys.stdin.readline()
                if key:
                    print('key')
                    return key
                else: # an empty line means stdin has been closed
                    print('eof')
            else:
                print('return -1')
                #return -1
        except (KeyboardInterrupt, EOFError):
            print('KeyboardInterrupt')
            pass
@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)        
        
def looper():
    pub = rospy.Publisher('motor_control_drive', SabertoothMotor, queue_size=10)
    rospy.init_node('motor_tester', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    constantPower = 25
    while not rospy.is_shutdown():
        #key = ord(getch.kbhit())
        key = 0
        
        #key = ord(sys.stdin.read(1))
        key = getch()
        if not key:
            key = 0
        if key == 119: #w
            motors.driveForward(constantPower)
            rospy.loginfo("key %s pressed", key)
        elif key == 115: #s
            motors.driveBackward(constantPower)
            rospy.loginfo("key %s pressed", key)
        elif key == 97: #a
            motors.driveLeft(constantPower)
            rospy.loginfo("key %s pressed", key)
        elif key == 100: #d
            motors.driveRight(constantPower)
            rospy.loginfo("key %s pressed", key)
        elif key == 101: #e, right
            motors.driveBoth(constantPower,0)
            rospy.loginfo("key %s pressed", key)
        elif key == 113: #q, left
            motors.driveBoth(0, constantPower)
            rospy.loginfo("key %s pressed", key)
        elif key == 102: #f
            constantPower = min(constantPower+10, 80)
        elif key == 118: #v
            constantPower = max(constantPower-10, 0)
        else:
            rospy.loginfo("stop motors %s", key)
            motors.stop()
        rate.sleep()
        # data = SabertoothMotor()
        # if direction == "up":
            # data.motor = 1
            # data.power = 100
        # if direction == "down":
            # data.motor = 1
            # data.power = -100
        # if direction == "":
            # data.motor = 1
            # data.power = 0
        # rospy.loginfo("sending command")
        # pub.publish(data)
init()
