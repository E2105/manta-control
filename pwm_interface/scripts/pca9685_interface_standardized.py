#!/usr/bin/env python

import rospy
from vortex_msgs.msg import Pwm, Manipulator
from std_msgs.msg import Float64MultiArray, Int8MultiArray
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
        """
        This node writes PWM signals to the GPIO pins on an Adafruit PCA9685 PWM Board.

        Pin 0-7:    All 8 thrusters
        Pin 8:      Camera servo
        Pin 9:      LED Brightness
        """

        rospy.init_node('pwm_node')
        self.pwm_sub = rospy.Subscriber('pwm', Float64MultiArray, self.thrusterCallback, queue_size=1)
        self.dpad_sub = rospy.Subcriber('dpad', Int8MultiArray, self.servoCallback, queue_size=1)

        self.pca9685 = Adafruit_PCA9685.PCA9685()
        self.pca9685.set_pwm_freq(FREQUENCY)
        self.pca9685.set_all_pwm(0, 0)              # No output on initialization
        self.current_pwm = [0]*16                   # 16 PWM pins
        self.camera_pwm = 1500                      # Servo set to middle position (Looking straight forward)

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('Initialized for {0} Hz.'.format(FREQUENCY))


    def thrusterCallback(self, msg):

        if len(msg.pins) == len(msg.positive_width_us):
            for i in range(len(msg.pins)):
                if msg.positive_width_us[i] != self.current_pwm[msg.pins[i]]:
                    self.pca9685.set_pwm(msg.pins[i], PWM_ON, self.microsecs_to_bits(msg.positive_width_us[i]))
                    self.current_pwm[msg.pins[i]] = msg.positive_width_us[i]

        print(msg.positive_width_us) #troubleshooting
        print(msg.pins)              #troubleshooting


    def servoCallback(self, msg):

        servo_tilt = msg.vertical_stepper_direction
        if servo_tilt == 1:
            self.camera_pwm += 5
        elif servo_tilt == -1:
            self.camera_pwm -= 5

        self.pca9685.set_pwm(8, PWM_ON, self.microsecs_to_bits(self.camera_pwm))
            

    def microsecs_to_bits(self, microsecs):
        duty_cycle_normalized = microsecs / PERIOD_LENGTH_IN_MICROSECONDS
        return int(round(PWM_BITS_PER_PERIOD * duty_cycle_normalized)


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
