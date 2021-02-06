#!/usr/bin/env python
import rospy
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy

class JoystickRemapper():