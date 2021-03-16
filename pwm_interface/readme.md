# The PWM Interface

This is the interface between the physical PWM module and the output of the controller.

Pulse Width Modulation (PWM) is used to control motors through an Electronic Speed Controller (ESC), and usually ranges from 1000 to 2000 microseconds from none to max thrust. The T200 thrusters work best as bidirectional, and will have no thrust at 1500 microseconds. This can be configured on the ESC with the help of a microcontroller like an Arduino. To control 8 thrusters and other PWM-controlled hardware we're using the Adafruit PCA9685 Servo Board.

## Library

We're using the official Adafruit library for the PWM board which can be found here: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library

## ROS

The PWM signals are stored in an array and sent to the physical hardware through a for loop.
