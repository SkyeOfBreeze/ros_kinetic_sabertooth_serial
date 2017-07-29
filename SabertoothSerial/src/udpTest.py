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
import socket, time
from std_msgs.msg import String
from time import sleep

direction = ""
UDP_IP = "127.0.0.1"
UDP_PORT = 30303
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('8.8.8.8', 0))  # connecting to a UDP address doesn't send packets
local_ip_address = s.getsockname()[0]
print "IP is ", local_ip_address
sock = socket.socket(socket.AF_INET, # Internet
socket.SOCK_DGRAM) # UDP
sock.bind((local_ip_address, UDP_PORT))


def init():
    if __name__ == '__main__':
        try:
            looper()
        except rospy.ROSInterruptException:
            pass
def looper():
    global sock
    pub = rospy.Publisher('motor_control_drive_raw', String, queue_size=10)
    rospy.init_node('motor_tester', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():#
        sock.settimeout(.3)
        try:
            data, addr = sock.recvfrom(2) # buffer size is 2 bytes
        except Exception:
            data = bytearray(chr(0))
        sock.settimeout(None)
        dataToSend = data
        rospy.loginfo("sending command")
        pub.publish(str.encode(dataToSend))
        rate.sleep()
init()