#!/usr/bin/env python3

import rospy
from vortex_msgs.msg import Pwm, Manipulator
import Adafruit_PCA9685

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
        self.sub_tilt = rospy.Subscriber('manipulator', Manipulator, self.callback, queue_size=1) # servo

        self.pca9685 = Adafruit_PCA9685.PCA9685()
        self.pca9685.set_pwm_freq(FREQUENCY)
        self.pca9685.set_all_pwm(0, 0)
        self.current_pwm = [0]*16

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('Initialized for {0} Hz.'.format(FREQUENCY))

    def callback(self, msg):
        # servo value
        camera_pwm = 1500

        if len(msg.pins) == len(msg.positive_width_us):
            for i in range(len(msg.pins)):
                if msg.positive_width_us[i] != self.current_pwm[msg.pins[i]]:
                    self.pca9685.set_pwm(msg.pins[i], PWM_ON, self.microsecs_to_bits(msg.positive_width_us[i]))
                    self.current_pwm[msg.pins[i]] = msg.positive_width_us[i]

        print(msg.positive_width_us) #troubleshooting
        print(msg.pins)              #troubleshooting

        # Trying out the tilt control
        servo_tilt = msg.vertical_stepper_direction
        if servo_tilt == 1:
            camera_pwm += 10
        elif servo_tilt == -1:
            camera_pwm -= 10

        self.current_pwm[msg.pins[8]] = camera_pwm
            


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
