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

# Packetized Serial Motor Driver for the SaberTooth Motor Drivers

import rospy, serial
from std_msgs.msg import String
from SabertoothSerial.msg import SabertoothMotor


class Packet:
    addr = -1
    command = -1
    data = -1
    checksum = -1

    def create(self):
        self.checksum = self.addr + self.command + self.data & 0b01111111

    def to_array(self):
        return bytearray([self.addr, self.command, self.data, self.checksum])

    def __init__(self, _addr, _command, _data):
        self.addr = _addr
        self.command = _command
        self.data = _data
        self.create()


# Possible Motor Driver channels 0-7, please look at Sabertooth Motor Driver Documentation
MOTOR_DRIVER_ADDRESS = [128, 129, 130, 131, 132, 133, 134, 135]

# Setup usb serial communication. If you have multiple usb serial devices, this may need to be changed.
# This cannot detect which one is the SaberTooth
ard = serial.Serial('/dev/ttyUSB0', 19200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)


# required to call this after a 2 second delay from motor power on
def start():
    # send out this command to set the baud. Documentation says to wait 2 seconds before invoking this command.
    # If the motor controller gets reset, this must be run again
    ard.write(0xAA)


def motor_raw(addr, motor, power):
    command = -1
    if motor == 1:
        if power > 0:
            command = 0
        else:
            command = 1
    else:
        if motor == 2:  #
            if power > 0:
                command = 4
            else:
                command = 5
        else:
            rospy.logerr("sabertooth_driver: Attempt to use a motor other than 1 or 2!")
            return False
    data = Packet(_addr=addr, _command=command, _data=abs(power))
    ard.write(data.to_array())
    return True


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.power)
    motor_raw(MOTOR_DRIVER_ADDRESS[0], data.motor, data.power)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sabertooth_driver', anonymous=True)

    rospy.Subscriber('motor_control_drive', SabertoothMotor, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    start()
    listener()
