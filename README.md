# The Manta Control System

## Summary

Used with:
* vortex_msgs
* xsens_mti_driver
* battery-monitor

## Background

Manta is an ROV designed and developed by Vortex NTNU, a student organization at NTNU Trondheim. It is able to move in 6 degrees of freedom with a total of 8 thrusters, half of them thrusting vertically, and the other half horizontally. It has a uniquely flat design similar to a manta ray, hence the name, and making it very stable control-wise.

This control system has taken inspiration from the Vortex NTNU github found at https://github.com/vortexntnu. This variant of the system has been developed in close cooperation with the organization.

## Theory

The control system is designed as a straight-forward SISO feedback system even though it has 8 actuators. The actual allocation of forces is done in a node after the controller. Much of the dynamics for our specific vehicle has been covered in a master thesis by Kristoffer Solber. [LINK?]

## The Open Loop

In normal ROV operation the setpoint is set by the connected joystick. We're running the default package **joy** on the shoreside computer to read the inputs from an Xbox controller. The **joystick_interface** remaps all the button clicks to what is considered smooth and straightforward control. This goes striaght into the controller.

The **controller**'s inputs are the from the joystick and the sensors. It has several mode to control depth and heading if needed. The output of the controller is forces and torque in 3 dimensions to get the drone, represented as a body frame, to where it wants it to be. This is the input of the **allocator** which allocates the forces to each thruster based on their placement. Ideally, this is the only thing one should need to change if the control system were to be used on another drone.

The **thruster_interface** interpolates the forces for each thrusters and converts it to a PWM signal between 1100 and 1900 us. The **pca9865_interface** writes this to the right GPIO pin connected to the ESC's. The functions of these two nodes are similar, but are kept separate for testing purposes.

## The Feedback Loop

Each sensor has a driver. The Bar30 pressure sensor node is created from its Adafruit library, and the IMU uses the **mti_xsens_driver**. Both these two are sent to an observer/estimator to be merged to a single signal for the controller.

## Other functions

Camera tilt and light brightness are controlled manually from the output of the **joystick_interface**.

The **vortex** package contains all the relevant configuration and launch files for our system.

# Implementation on the Shoreside Computer

## Ubuntu Desktop 18.04 LTS

We are using Ubuntu 18.04 and ROS Melodic on the shoreside computer. This PC runs the GUI and graphical tools if needed. Most useful tools are only officially supported up to ROS Melodic, but usually works on ROS Noetic.

## 1. ROS Melodic

Detailed walkthrough: http://wiki.ros.org/melodic/Installation/Ubuntu

Quick setup (Only follow this if you know what each command does):

1. Sources and keys

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

3. Install ROS

Grabbing the full desktop versions since a normal laptop should be able to handle it well.

```
sudo apt update
```

```
sudo apt install ros-melodic-desktop-full
```

4. Environment variables

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

5. Dependancies

```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

5 rosdep

```
sudo apt install python-rosdep
```

```
sudo rosdep init
rosdep update
```

## 2. Other dependancies

2.1 To connect to the webviz GUI

```
sudo apt install ros-melodic-rosbridge-server
```

2.2 The joystick package

```
sudo apt install ros-melodic-joy
```

2.3 Other dependancies

```
sudo apt install protobuf-compiler ros-melodic-message-to-tf ros-melodic-geographic-msgs ros-melodic-move-base ros-melodic-move-base-msgs
```

## 3. ROS Workspace

1. Catkin tools

```
sudo apt-get update
sudo apt-get install python-catkin-tools
```

2. The catkin workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

3. Build the workspace

```
cd ~/catkin_ws
catkin build
```

3. Source the workspace

```
echo 'source $HOME/catkin_ws/devel/setup.bash' >> ~/.bashrc
```


## Custom Message Class for the Water Linked Underwater GPS

We are mostly using default messages, but to work well with the Water Linked system a custom emssage was created.

1. Enter the package folder and clone the repository from github

```
cd ~/catkin_ws/src
git clone https://github.com/vortexntnu/vortex_msgs.git
```

2. Build the packages

```
cd ~/catkin_ws
catkin build
```



# Implementation on the Raspberry Pi 4

The entire control system runs on a Raspberry Pi 4 4GB on the drone. We have chosen Ubuntu Mate as the OS to keep the ubuntu functionality while having access to the GPIO pins without any hassle. The OS runs slower than normal Ubuntu server, and only the newest version of Ubuntu is available (20.04 LTS). This is an awkward middleground since much of the tools we're using in ROS are only officially supported on version 18.04 of Ubuntu, and many developers has already begun to work on ROS2. But thanks to the modular strength of ROS it works out.

## Setup

1. Ubuntu Mate

Write Ubuntu Mate 20.04 LTS to a SD card and boot the Raspberry Pi. It will try to upgrade all software to newest versions, which will take a while, so a wired internet connection should be considered. You cannot install other programs while this is happening and the terminal will just tell you that the thread is used for *unattended-upgr*.

Remember that Ubuntu 20.04 uses Python 3 by default, and earlier Ubuntu versions uses Python 2. Dependancies regarding this is covered further down.

2. All the basics

2.1 Activate SSH with OpenSSH:

```
sudo apt update
```
```
sudo apt install openssh-server
```

Enable SSH.

```
sudo systemctl enable ssh
```

```
sudo ufw allow ssh
```

Verify that it's running with this:

```
sudo systemctl status ssh
```


2.2 Configure a static IP

It is always ann advantage to change to a static IP on the same subent as your other devices it will communicate with. On Ubuntu Mate you need a display, keyboard and mouse to change the settings like a normal dekstop PC. If you absolutely need a DHCP to get an internet connection, one can for example have wifi with DHCP, and static IP on ethernet.


2.3 Install ROS

Only install *ros-base* on the Pi since the graphical tools isn't really needed and takes up unecessary space.

http://wiki.ros.org/noetic/Installation/Ubuntu

Install rosdep too:

```
sudo apt install python3-rosdep
```

```
sudo rosdep init
rosdep update
```

Configure the workspace using catkin tools.

We are using *catkin tools* to work with our workspace.

```
sudo apt install python3-catkin-tools
```

Create the workspace and intialize it.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
```

2.4 Install general dependancies

Some generally needed dependancies are not installed by default. This is either because *ros-base* excludes some specific packages, or that it is installed on an ARM architecture.

Package: "osrf-common"

```
sudo apt install python3-catkin-tools python3-osrf-common
```

Run Python 3 as Python 2: "python-is-python3"

```
sudo apt install python-is-python3
```

Programs for configuring the GPIO pins (Might not be needed)

```
sudo apt install i2c-tools
```

Git for version control

```
sudo apt install git
```

### Camera

Camera packages for ROS

```
sudo apt-get install ros-noetic-image-transport ros-noetic-camera-info-manager libavcodec-dev libswscale-dev
```

Video4Linux utilities

```
sudo apt-get install v4l-utils
```

Viewing the image produced

```
sudo apt-get install ros-noetic-image-view
```

The camera package

```
sudo apt install ros-noetic-usb-cam
```

### Needed for the controller

```
sudo apt install ros-noetic-eigen-conversions
```

```
sudo apt install ros-noetic-tf
```

```
sudo apt install ros-noetic-roslint
```

The default joystick package

```
sudo apt install ros-noetic-joy
```

### GPIO Libraries

Install these in the home folder.

"Adafruit_Python_PCA9685" and "Adafruit_Python_GPIO".


4. Clone the custom message system.

```
cd ~/catkin_ws/src
git clone git clone https://github.com/vortexntnu/vortex_msgs.git
```

5. Clone the control system.

```
cd ~/catkin_ws/src
git clone https://github.com/USERNAME/REPO.git
```

6. Build the workspace. Build the message package first if any other package is dependant on it.

```
cd ~/catkin_ws
catkin build vortex_msgs
catkin build
```

7. Run some tests

Launch files.
