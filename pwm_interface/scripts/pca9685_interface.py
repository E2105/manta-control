#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray, Int8MultiArray
import Adafruit_PCA9685

"""
This node writes PWM signals to the GPIO pins on a ADAFRUIT PCA9685 PWM board.

Pin 0 - 7
    Connected to two 4-in-1 ESCs controlling 8 thrusters.
Pin 8
    Controls the camera servo allowing for tilt control.
Pin 9
    Controls the brightness of the lights.
"""

# ROV Constants
THRUST_RANGE_LIMIT = 100
NUM_THRUSTERS = rospy.get_param('/propulsion/thrusters/num')
THRUST_OFFSET = rospy.get_param('/thrusters/offset')
LOOKUP_THRUST = rospy.get_param('/thrusters/characteristics/thrust')
LOOKUP_PULSE_WIDTH = rospy.get_param('/thrusters/characteristics/pulse_width')
THRUSTER_MAPPING = rospy.get_param('/propulsion/thrusters/map')
THRUSTER_DIRECTION = rospy.get_param('/propulsion/thrusters/direction')

# PWM Board Constants
PWM_BITS_PER_PERIOD = 4095 #rospy.get_param('/pwm/counter/max')
FREQUENCY = 50.0 #rospy.get_param('/pwm/frequency/set')
FREQUENCY_MEASURED = 51.6 #rospy.get_param('/pwm/frequency/measured')
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0 / FREQUENCY_MEASURED
PWM_ON = 0  # Start of duty cycle

class Pca9685InterfaceNode(object):
    def __init__(self):
        """
        This node writes PWM signals to the GPIO pins on an Adafruit PCA9685 PWM Board.

        Pin 0-7:    All 8 thrusters
        Pin 8:      Camera servo
        Pin 9:      LED Brightness
        """

        rospy.init_node('pwm_node')
        self.pwm_sub = rospy.Subscriber('pwm_values', UInt16MultiArray, self.thrusterCallback, queue_size=1)
        self.dpad_sub1 = rospy.Subscriber('joy/dpad', Int8MultiArray, self.servoCallback, queue_size=1)
        self.menu_sub = rospy.Subscriber('joy/menu_buttons', Int8MultiArray, self.ledCallback, queue_size=1)

        self.pca9685 = Adafruit_PCA9685.PCA9685()
        self.pca9685.set_pwm_freq(FREQUENCY)
        self.pca9685.set_all_pwm(0, 0)              # No output on initialization
        self.current_pwm = [0]*16                   # 16 PWM pins
        self.camera_pwm = 1500                      # Servo set to middle position (Looking straight forward)
        self.brightness = 0
        self.cam_pin = 8
        self.led_pin = 9
        self.is_on = False
        self.previous = 0

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('Initialized for {0} Hz.'.format(FREQUENCY))


    def thrusterCallback(self, msg):
        # Writes the PWM signal to the GPIO pin
        # msg: the array of PWM signals
        
        if NUM_THRUSTERS == len(msg.data):
            for pin in range(NUM_THRUSTERS):
                if msg.data[pin] != self.current_pwm[pin]:
                    self.pca9685.set_pwm(pin, PWM_ON, self.microsecs_to_bits(msg.data[pin]))
                    self.current_pwm[pin] = msg.data[pin]


    def servoCallback(self, msg):
        # dpad.data[0] --> horizontal dpad
        # dpad.data[1] --> vertical dpad
        dpad = msg.data

        if dpad[1] == 1:
            if self.camera_pwm > 2000:
                self.camera_pwm = 2000
            else:
                self.camera_pwm += 100
                
        if dpad[1] == -1:
            if self.camera_pwm < 1000:
                self.camera_pwm = 1000
            else:
                self.camera_pwm -= 100

        self.pca9685.set_pwm(self.cam_pin, PWM_ON, self.microsecs_to_bits(self.camera_pwm))


    def ledCallback(self, msg):
        # dpad.data[0] --> horizontal dpad
        # dpad.data[1] --> vertical dpad
        menu = msg.data

        if menu[1] == 1 and self.previous == 0:
            if self.is_on == True:
                self.is_on = False
            else:
                self.is_on = True
                
        if self.is_on:
            self.brightness = 1900
        else:
            self.brightness = 1100
            
        self.previous = menu[1]

        self.pca9685.set_pwm(self.led_pin, PWM_ON, self.microsecs_to_bits(self.brightness))

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(PWM_BITS_PER_PERIOD * duty_cycle_normalized))

    def shutdown(self):
        # Stop output if node shuts down
        self.pca9685.set_all_pwm(0, 0)

if __name__ == '__main__':
    try:
        pwm_node = Pca9685InterfaceNode()
        rospy.spin()
    except IOError:
        rospy.logerr('IOError caught. Shutting down PWM node.')
    except rospy.ROSInterruptException:
        pass
