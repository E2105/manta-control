#!/usr/bin/env python

import rospy
from math import isnan, isinf
import numpy as np

from vortex_msgs.msg import ThrusterForces
from std_msgs.msg import UInt16MultiArray, Float64MultiArray

# Parameters
THRUST_RANGE_LIMIT = 100
NUM_THRUSTERS = rospy.get_param('/propulsion/thrusters/num')
THRUST_OFFSET = rospy.get_param('/thrusters/offset')
LOOKUP_THRUST = rospy.get_param('/thrusters/characteristics/thrust')
LOOKUP_PULSE_WIDTH = rospy.get_param('/thrusters/characteristics/pulse_width')
THRUSTER_MAPPING = rospy.get_param('/propulsion/thrusters/map')
THRUSTER_DIRECTION = rospy.get_param('/propulsion/thrusters/direction')

"""
The function of this node is currently shouldn't need to be its own node AS PER NOW.

1. Either merge it with the PWM interface

OR

2. Use it as a intermediary for logging and arming of thrusters
"""

class ThrusterInterface(object):

    def __init__(self):

        # Initialization
        rospy.init_node('thruster_interface', anonymous=False, log_level=rospy.INFO)

        # Topics
        #self.sub = rospy.Subscriber('thruster_forces', ThrusterForces, self.callback)
        self.sub = rospy.Subscriber('thruster_forces', Float64MultiArray, self.callback)
        self.pub_pwm = rospy.Publisher('pwm_values', UInt16MultiArray, queue_size=10)

        # Startup/Shutdown
        self.output_to_zero()                       # Send stop signal on start
        rospy.on_shutdown(self.output_to_zero)      # Send stop signal on shutdown

        # Logging
        rospy.loginfo('Initialized with thruster direction:\n\t{0}.'.format(THRUSTER_DIRECTION))

        for i in range(NUM_THRUSTERS):
            THRUST_OFFSET[i] *= THRUSTER_DIRECTION[i]

        rospy.loginfo('Initialized with offset:\n\t{0}.'.format(THRUST_OFFSET))


    def callback(self, msg):
        # Converts needed thrust calculated by the controller to a PWM signal
        if not self.healthy_message(msg):
            return

        # Initializing
        thrust = list(msg.thrust)                      # Topic: "thruster_forces"
        pwm_values = UInt16MultiArray()                     # Topic: "pwm_values"

        microsecs = [None] * NUM_THRUSTERS

        for i in range(NUM_THRUSTERS):
            microsecs[i] = self.thrust_to_microsecs(thrust[i] + THRUST_OFFSET[i])
            pwm_values.data.append(microsecs[i])
            
        self.pub_pwm.publish(pwm_values)


    def output_to_zero(self):
        # Publishes a stop signal to all thrusters
        neutral_pwm = self.thrust_to_microsecs(0)
        pwm_values = UInt16MultiArray()

        for i in range(NUM_THRUSTERS):
            pwm_values.data.append(neutral_pwm)
            
        self.pub_pwm.publish(pwm_values)


    def thrust_to_microsecs(self, thrust):
        # Converts wanted thrust to PWM by interpolating two parameter lists
        # Output: 1100 - 1900 microseconds
        return np.interp(thrust, LOOKUP_THRUST, LOOKUP_PULSE_WIDTH)


    def healthy_message(self, msg):
        if len(msg.thrust) != NUM_THRUSTERS:
            rospy.logwarn_throttle(10, 'Wrong number of thrusters, ignoring...')
            return False

        for t in msg.thrust:
            if isnan(t) or isinf(t) or (abs(t) > THRUST_RANGE_LIMIT):
                rospy.logwarn_throttle(10, 'Message out of range, ignoring...')
                return False

        return True


if __name__ == '__main__':
    try:
        thruster_interface = ThrusterInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
