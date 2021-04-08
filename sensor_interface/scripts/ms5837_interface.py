#!/usr/bin/env python

import rospy
import requests
import argparse
import json
import time

import ms5837
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# Library: https://github.com/bluerobotics/ms5837-python

class Ms5837InterfaceNode(object):
    """
    This node continuously publishes readings from a Bar30 pressure sensor.
    """

    def set_depth(url, depth, temp):        # Extarnaldepth
        payload = dict(depth=depth, temp=temp)
        r = requests.put(url, json=payload, timeout=10)

    
    def get_data(url):                      # GetPosition 
        r = requests.get(url)
        return r.json()
   
    def get_acoustic_position(base_url):    # GetPosition 
        return get_data("{}/api/v1/position/acoustic/filtered".format(base_url))
            
    
   

    def __init__(self):
        rospy.init_node('pressure_node')

        self.sensor = ms5837.MS5837_30BA()

        # Initiating Subscribers and Publishers
        self.pub_pressure = rospy.Publisher('sensors/pressure', FluidPressure, queue_size=1)
        self.pub_temp = rospy.Publisher('sensors/temperature', Temperature, queue_size=1)
        self.pub_position = rospy.Subscriber('estimator/state', Odometry, queue_size=1)
        self.sub_depth = rospy.Publisher('depth/state', Float64, queue_size=1)

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

        depth = Float64()       
        position_UGPS = Odometry.pose.pose.position() 


        parser = argparse.ArgumentParser(description=__doc__)   #Sets the IP-address of the UGPS
        parser.add_argument('-u', '--url', help='Base URL to use', type=str, default='http://demo.waterlinked.com')
        args = parser.parse_args()
        base_url = args.url

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
            
            set_depth('{}/api/v1/external/depth'.format(base_url), depth, temp.temperature) #Externaldepth  // It says set_depth is not defined in vs code??
                                                                                            #               // but is defined at line 21
            
            data_ugps = get_acoustic_position(base_url)         #GetPosition
            position_UGPS.x(data_ugps['x'])
            position_UGPS.y(data_ugps['y'])
            position_UGPS.z(data_ugps['z'])
            self.pub_position.publish(position_UGPS)            
            
            rospy.Rate(10).sleep()  # 10 hz


if __name__ == '__main__':
    try:
        pressure_node = Ms5837InterfaceNode()
        rospy.spin()
    #except IOError:
        #rospy.logerr('IOError caught. Shutting down.')
    except rospy.ROSInterruptException:
        pass
