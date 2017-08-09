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

## Packetized Serial Motor Driver for the Sabertooth Motor Drivers

import rospy, serial, Queue
from std_msgs.msg import String
from SabertoothSerial.msg import SabertoothMotor

class SerialMotorControl:
    serialPort = '/dev/ttyUSB0'
    timeout = 0
    #Setup usb serial communication. If you have multiple usb serial devices, this may need to be changed. This cannot detect which one is the sabertooth
    ard = 0
    publishEnabled = False
    pub = 0
    queue = Queue.Queue()
    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def sendCommand(self, motor, power):
        if self.publishEnabled:
            self.publish_raw(motor, power)
        else:
            self.motor_raw_process(motor, power)
        
    def setPublishEvent(self, publish):
        self.publishEnabled = publish
        if publish:
            rospy.init_node('motor_tester', anonymous=True)
            self.pub = rospy.Publisher('motor_control_drive', SabertoothMotor, queue_size=10)
        else:
            self.ard = serial.Serial(self.serialPort, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        
    def setSerialPort(self, port):
        self.serialPort = port
        if not self.publishEnabled:
            self.ard = serial.Serial(self.serialPort, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        
    def publish_raw(self, motor, power):
        data = SabertoothMotor(motor, power)
        data.motor = motor
        data.power = power
        self.pub.publish(data)
      
    def getByteOfMotor(self, motor, power):
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
      data = self.getByteOfMotor(motor, power)
      self.motor_raw(data)
      
    def motor_raw(self, data):
      self.ard.write(chr(data))
      
    def driveBoth(self, leftPower, rightPower):
        self.sendCommand(0, leftPower)
        self.sendCommand(1, rightPower)
        
    def drive(self, power):
        self.driveBoth(power, power)
        
    def driveForward(self, power):
        self.drive(power)

    def driveBackward(self, power):
        self.drive(-power)
    
    def driveLeft(self, power):
        self.driveBoth(-power, power)
    def driveRight(self, power):
        self.driveBoth(power, -power)
        
    def stop(self):
        self.drive(0)
        
    def callback(self, data):
        self.queue.put(data)
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.power)
        
    def raw_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard raw data')
        self.ard.write(str.decode(data))

    def listener(self):
        #don't let this run unless it is a node. Better Idea; don't allow this to be a node
        if __name__ == '__main__':
            # In ROS, nodes are uniquely named. If two nodes with the same
            # name are launched, the previous one is kicked off. The
            # anonymous=True flag means that rospy will choose a unique
            # name for our 'listener' node so that multiple listeners can
            # run simultaneously.
            rospy.init_node('sabertooth_driver', anonymous=True)

            rospy.Subscriber('motor_control_drive', SabertoothMotor, self.callback)
            rospy.Subscriber('motor_control_drive_raw', String, self.raw_callback)
            timeSinceLastMessage = rospy.get_time()
            while not rospy.is_shutdown():
                while not self.queue.empty():
                    timeSinceLastMessage = rospy.get_time()
                    data = self.queue.get()
                    self.motor_raw_process(data.motor, data.power)
                    data = 0
                else:
                    self.timeout = self.timeout + 1
                    if rospy.get_time() - timeSinceLastMessage > 1:
                        self.stop()
            # spin() simply keeps python from exiting until this node is stopped
            rospy.spin()
        
if __name__ == '__main__':
    SerialControl = SerialMotorControl()
    SerialControl.setPublishEvent(False)
    try:
        SerialControl.listener()
    except rospy.ROSInterruptException:
        pass
    SerialControl.stop()
        
