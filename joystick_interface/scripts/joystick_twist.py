#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import ByteMultiArray, Int8MultiArray
from geometry_msgs.msg import Twist


class JoystickInterfaceNode():

    """
    This class maps the joystick input.
    """

    def __init__(self):
        rospy.init_node('joystick_node')

        # Initiating Subcribers and Publishers
        # ------------------------------------

        # Joystick Input
        self.sub = rospy.Subscriber('joy_throttle', Joy, self.callback, queue_size=1)

        # Motion data
        self.pub_motion = rospy.Publisher('joy/twist_motion', Twist, queue_size=1)

        # Control modes
        self.pub_mode = rospy.Publisher('joy/control_mode', ByteMultiArray, queue_size=1)

        # D-PAD
        self.pub_dpad = rospy.Publisher('joy/dpad', Int8MultiArray, queue_size=1)


        # Input Mapping
        # -------------

        self.buttons_map = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'back',
                            'start', 'power', 'stick_button_left',
                            'stick_button_right']

        self.axes_map = ['horizontal_axis_left_stick',
                         'vertical_axis_left_stick',
                         'LT',
                         'horizontal_axis_right_stick',
                         'vertical_axis_right_stick',
                         'RT',
                         'dpad_horizontal',
                         'dpad_vertical']

    def callback(self, msg):
        
        # Instances of published classes
        twist = Twist()             # Motion
        bytez = ByteMultiArray()    # Modes
        dpad = Int8MultiArray()     # D-PAD

        # Dictionaries with button/axes names as keys
        twist_motion = {}
        button_array = {}

        for i in range(len(msg.axes)):
            twist_motion[self.axes_map[i]] = msg.axes[i]

        for j in range(len(msg.buttons)):
            button_array[self.buttons_map[j]] = msg.buttons[j]

        # Motion Controls
        # ---------------

        # Linear motion
        twist.linear.x = twist_motion['vertical_axis_left_stick']       # Surge
        twist.linear.y = -twist_motion['horizontal_axis_left_stick']    # Sway
        twist.linear.z = (twist_motion['RT'] - twist_motion['LT'])/2    # Heave

        # Angular motion
        twist.angular.x = (button_array['RB'] - button_array['LB'])     # Roll
        twist.angular.y = -twist_motion['vertical_axis_right_stick']    # Pitch
        twist.angular.z = twist_motion['horizontal_axis_right_stick']   # Yaw


        # Control Modes
        # -------------

        bytez.data = [
            button_array['A'],
            button_array['X'],
            button_array['B'],
            button_array['Y']
        ]

        """
        bytez is not pushing boolean values that the controller EXPECTS, this needs to be done in the controller
        """
        
        # The D-PAD
        # ---------

        dpad.data = [
            twist_motion['dpad_horizontal'],
            twist_motion['dpad_vertical']
        ]

        # Publishing topics
        self.pub_motion.publish(twist)
        self.pub_mode.publish(bytez)
        self.pub_dpad.publish(dpad)


if __name__ == '__main__':
    try:
        joystick_node = JoystickInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
