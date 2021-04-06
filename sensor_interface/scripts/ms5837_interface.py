#!/usr/bin/env python

import rospy

import ms5837
from sensor_msgs.msg import FluidPressure, Temperature

# Library: https://github.com/bluerobotics/ms5837-python

class Ms5837InterfaceNode(object):
    """
    This node continuously publishes readings from a Bar30 pressure sensor.
    """

    def __init__(self):
        rospy.init_node('pressure_node')

        self.sensor = ms5837.MS5837_30BA()

        # Initiating Subscribers and Publishers
        self.pub_pressure = rospy.Publisher('sensors/pressure', FluidPressure, queue_size=1)
        self.pub_temp = rospy.Publisher('sensors/temperature', Temperature, queue_size=1)

        if not self.sensor.init():
            rospy.logfatal('Failed to initialize.')
        else:
            if not self.sensor.read():
                rospy.logfatal('Failed to read.')
            else:
                rospy.loginfo('Successfully initialized.')
         
        self.talker()

    def talker(self):
        
        # Class Instances
        pressure = FluidPressure()
        pressure.variance = 0
        temp = Temperature()
        temp.variance = 0

        while not rospy.is_shutdown():
            if self.sensor.read():
                # Pressure reading (Pa)
                pressure.header.stamp = rospy.get_rostime()
                pressure.fluid_pressure = self.sensor.pressure(ms5837.UNITS_Pa)

                # Temperature reading (C)
                temp.header.stamp = rospy.get_rostime()
                temp.temperature = self.sensor.temperature(ms5837.UNITS_Centigrade)

                # Publishing
                self.pub_pressure.publish(pressure)
                self.pub_temp.publish(temp)
            else:
                rospy.roswarn('Failed to read.')

            rospy.Rate(10).sleep()  # 10 hz


if __name__ == '__main__':
    try:
        pressure_node = Ms5837InterfaceNode()
        rospy.spin()
    #except IOError:
        #rospy.logerr('IOError caught. Shutting down.')
    except rospy.ROSInterruptException:
        pass
