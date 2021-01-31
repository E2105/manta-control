# Manta
A test.

"Manta" is the name of the ROV given by Vortex NTNU at the time of construction in 2017.
It has competed in ROV and AUV competitions internationally, and is now mainly used and maintained as a ROV.

# Description
This repo contains the necessary ROS packages for controlling the underwater drone physically.

## Test

## Launch files
Launch files are called with the `roslaunch` command and contains a specific set of packages meant to do a specific task. Usually differentiated between physical operation and simulator operation.


**Usage:**

`roslaunch   package_name   launch_file.launch`

**Files:**

simulator.launch

&nbsp;&nbsp;&nbsp;&nbsp; *Launches the barebone control structure necessary for simulator control of Manta.*
  
manta.launch

&nbsp;&nbsp;&nbsp;&nbsp; *Launches all the necessary packages for normal physical operation of Manta.*

## System Design

The function of the packages are designed as a traditional closed loop feedback system.

The measured states are the depth and orientation of the ROV determined by the setpoint given by the a joystick.

*Add the feedback loop image*.

### Packages

**joy_node:** Default node in ROS reading joystick input.

**joystick_interface:** Joystick mapping.

**controller:** Controller.

**allocator:** Allocates the forces. Also needed for the simulators thruster_manager.

**thruster_interface:** Determines each thruster's force needed.

**pwm_interface:** Sends PWM to all ESC's.

**sensor_node:** Reads raw sensor data. (Pressure and position)

**sensor_interface:** Processes and sends sensor data to the controller.

# Setup

## Prerequisite

Linux Ubuntu 18.04 with ROS Melodic.

&nbsp;&nbsp;&nbsp;&nbsp; Install ROS Melodic here:

The Manta ROV Simulator.

&nbsp;&nbsp;&nbsp;&nbsp; Follow the instructions here:

The custom messages library/package:

&nbsp;&nbsp;&nbsp;&nbsp; vortex_msgs


## Dependencies

The catkin tools and some python dependancies are needed.

## Downloading

git clone

## Building

catkin build

## Test functionality

## Camera node

I'm using the default usb_cam node from ROS with a few modifications.

New launch file with correct aspect ratio and len/wid.
