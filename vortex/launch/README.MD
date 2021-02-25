# Launch files

## Description

Launch files are parametric files telling ROS which nodes to run. On the ground level, you can easily reference the necessary nodes and they will connect with each other as longas their topics are related.

It is a highly modular form of referencing, and can be used to decide many things like start positions or parameters for the controller.

## Usage

```
roslaunch package_name launch_file.launch
```

## Relevant files

### pwmtest.launch

This activates the whole control system with functioning control of physical thrusters. This can only be run on the actual Raspberry Pi 4 because of the I2C dependencies.

### manta.launch

Depricated ROV launch file using rosbridge.

### simulator.launch

Launches the needed control system to steer the drone manually in the simulator.
