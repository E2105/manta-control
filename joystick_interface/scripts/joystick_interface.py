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
        
        # MENU
        self.pub_menu = rospy.Publisher('joy/menu_buttons', Int8MultiArray, queue_size=1)


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
        self.button = []
        self.previous =[]
        self.counter = 0
        

    def callback(self, msg):
        
        # Instances of published classes
        twist = Twist()             # Motion
        bytez = ByteMultiArray()    # Modes
        dpad = Int8MultiArray()     # D-PAD
        menu = Int8MultiArray()     # MENU

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

        
        # -------------

        if button_array['X'] == 1 and self.previous[0] == 0:
            if self.button[0] = True:
                self.button[0] = False
            else:
                self.button[0] = True
        self.previous[0] = button_array['X']
        
        if button_array['Y'] == 1 and self.previous[1] == 0:
            if self.button[1] = True:
                self.button[1] = False
            else:
                self.button[1] = True
        self.previous[1] = button_array['Y']
        
        if button_array['A'] == 1 and self.previous[2] == 0:
            if self.button[2] = True:
                self.button[2] = False
            else:
                self.button[2] = True
        self.previous[2] = button_array['A']
        
        if button_array['B'] == 1 and self.previous[3] == 0:
            if self.button[3] = True:
                self.button[3] = False
            else:
                self.button[3] = True
        self.previous[3] = button_array['B']
        
        
        
        
        if self.button[0]
            if self.counter == 5 or self.counter == 6
                self.counter = 0
            else
                self.counter += 1
            
        if self.button[1]
            if self.counter == 0 or self.counter == 6
                self.counter = 5
            else
                self.counter -= 1
        
        if self.button[2]
            self.counter = 0
        if self.button[3]
            self.counter = 6
        
        for i in range(6):
            if i == self.counter
                bytez.data[i] = 1
            else 
                bytez.data[i] = 0

        """
        bytez is not pushing boolean values that the controller EXPECTS, this needs to be done in the controller
        """
        
        # The D-PAD
        # ---------

        dpad.data = [
            int(twist_motion['dpad_horizontal']),
            int(twist_motion['dpad_vertical'])
        ]
        
        # The MENU BUTTONS
        # ---------

        menu.data = [
            int(button_array['back']),
            int(button_array['start']),
            int(button_array['power'])
        ]

        # Publishing topics
        self.pub_motion.publish(twist)
        self.pub_mode.publish(bytez)
        self.pub_dpad.publish(dpad)
        self.pub_menu.publish(menu)


if __name__ == '__main__':
    try:
        joystick_node = JoystickInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
