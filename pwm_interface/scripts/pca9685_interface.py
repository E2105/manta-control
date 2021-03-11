#!/usr/bin/env python3

import rospy
from vortex_msgs.msg import Pwm, Manipulator
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

# Constants
PWM_BITS_PER_PERIOD = 4095 #rospy.get_param('/pwm/counter/max')
FREQUENCY = 50.0 #rospy.get_param('/pwm/frequency/set')
FREQUENCY_MEASURED = 51.6 #rospy.get_param('/pwm/frequency/measured')
PERIOD_LENGTH_IN_MICROSECONDS = 1000000.0 / FREQUENCY_MEASURED
PWM_ON = 0  # Start of duty cycle

class Pca9685InterfaceNode(object):
    def __init__(self):
        rospy.init_node('pwm_node')
        self.sub = rospy.Subscriber('pwm', Pwm, self.callback, queue_size=1)
        self.sub2 = rospy.Subscriber('manipulator_command', Manipulator, self.callback_tilt) # servo

        self.pca9685 = Adafruit_PCA9685.PCA9685()
        self.pca9685.set_pwm_freq(FREQUENCY)
        self.pca9685.set_all_pwm(0, 0)
        self.current_pwm = [0]*16
        self.camera_pwm = 1500

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('Initialized for {0} Hz.'.format(FREQUENCY))

    def callback(self, msg):
        # servo value

        if len(msg.pins) == len(msg.positive_width_us):
            for i in range(len(msg.pins)):
                if msg.positive_width_us[i] != self.current_pwm[msg.pins[i]]:
                    self.pca9685.set_pwm(msg.pins[i], PWM_ON, self.microsecs_to_bits(msg.positive_width_us[i]))
                    self.current_pwm[msg.pins[i]] = msg.positive_width_us[i]

        print(msg.positive_width_us) #troubleshooting
        print(msg.pins)              #troubleshooting

    def callback_tilt(self, msg):
        # Trying out the tilt control
        servo_tilt = msg.vertical_stepper_direction
        if servo_tilt == 1:
            self.camera_pwm += 5
        elif servo_tilt == -1:
            self.camera_pwm -= 5

        self.pca9685.set_pwm(8, PWM_ON, self.microsecs_to_bits(self.camera_pwm))
            

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(PWM_BITS_PER_PERIOD * duty_cycle_normalized))

    def remap(self, old_val, old_min, old_max, new_min, new_max):
        # Remaps an interval. Used for testing purposes.
        return (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min

    def shutdown(self):
        self.pca9685.set_all_pwm(0, 0)

if __name__ == '__main__':
    try:
        pwm_node = Pca9685InterfaceNode()
        rospy.spin()
    except IOError:
        rospy.logerr('IOError caught, shutting down.')
    except rospy.ROSInterruptException:
        pass
