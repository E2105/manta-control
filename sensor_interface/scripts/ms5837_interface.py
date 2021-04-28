#!/usr/bin/env python

import rospy

import ms5837
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float64

# Library: https://github.com/bluerobotics/ms5837-python

ATM_PRESSURE_PA = rospy.get_param('/atmosphere/pressure')
LOCAL_GRAVITY = rospy.get_param('/gravity/acceleration')

print("GRAVITY YALL: " + str(LOCAL_GRAVITY))

rospy.loginfo("Node initialized with atmospheric pressure: %s", ATM_PRESSURE_PA)
rospy.loginfo("Node initialized with local gravity: %s", LOCAL_GRAVITY)

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
        self.pub_depth = rospy.Publisher('sensors/depth', Float64, queue_size=1)

        if not self.sensor.init():
            rospy.logfatal('Failed to initialize.')
        else:
            if not self.sensor.read():
                rospy.logfatal('Failed to read.')
            else:
                rospy.loginfo('Successfully initialized with pressure: ' + str(ATM_PRESSURE_PA) + ' and gravity: ' + str(LOCAL_GRAVITY))
         
        self.talker()

    def talker(self):
        
        # Class Instances
        pressure = FluidPressure()
        pressure.variance = 0
        temp = Temperature()
        temp.variance = 0
        depth = Float64()

        while not rospy.is_shutdown():
            if self.sensor.read():
                # Pressure reading (Pascal)
                pressure.header.stamp = rospy.get_rostime()
                pressure.fluid_pressure = self.sensor.pressure(ms5837.UNITS_Pa)

                # Temperature reading (Celsius)
                temp.header.stamp = rospy.get_rostime()
                temp.temperature = self.sensor.temperature(ms5837.UNITS_Centigrade)

                # Depth reading (Meters)
                depth.data = self.sensor.depth(ATM_PRESSURE_PA, LOCAL_GRAVITY)

                # Publishing
                self.pub_pressure.publish(pressure)
                self.pub_temp.publish(temp)
                self.pub_depth.publish(depth)
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
