# ros_kinetic_sabertooth_serial
Ros Library/Node to communicate with a Sabertooth Motor Controller via RS-232 Serial.

Documentation in the beginning may be lacking, but will get better over time. This currently has a somewhat working simple serial running at 9600 on /dev/ttyUSB0, that can be either a listener node or be used via a different node as a dependency. Packetized serial does not work at the moment


Note! Currently unstable! Use at your own risk! As I finalize the structure, things may break, or not work at all when you check it out

# Platforms
## linux
yes
## windows
ros does not support windows

# Languages
## python 2.7 linux
this is the language of all the source files so far. What may make more sense for future revisions is to make a python only motor controller libary, but for now, it is bundled in the ros package

## c++
Will be added at a later date

# Compatible Sabertooth controllers
More or all that support serial may be compatible, but the list below shows what they are tested on. Feel free to notify me of other working devices

- Sabertooth 2x25
