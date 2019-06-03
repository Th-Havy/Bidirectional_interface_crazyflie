![EPFL logo](https://github.com/Th-Havy/Bidirectional_interface_crazyflie/blob/master/readme_images/epfl_logo.png)
![LIS logo](https://github.com/Th-Havy/Bidirectional_interface_crazyflie/blob/master/readme_images/logo_LIS.png)

# Bidirectional interface - hardware implementation with crazyflie

This repository contains the code for the hardware implementation of the [Bidirectional hand interface](https://github.com/AntoineWeber/Bidirectional_Interface) previously developed in simulation. This was implemented for a semester project in the LIS lab at EPFL.

The hardware platform used is the [Crazyflie 2.1](https://www.bitcraze.io/crazyflie-2-1/) drone, and the software platform is [Crazyswarm](https://github.com/USC-ACTLab/crazyswarm) (from which this repository is forked).

**TODO: add gif to show results**

## Setup

Clone this repository
```bash
git clone https://github.com/Th-Havy/Bidirectional_interface_crazyflie
cd Bidirectional_interface_crazyflie
```

The setup is the same than for crazyswarm and is detailled in this [document](https://docs.google.com/document/d/16PeWJRykn29gsYg2IYSfiZUJEAPd9E132CeRZWg4XbU/edit?usp=sharing).

Note that this repository does not make modifications to crazyswarm, the only difference is the created nodes and launch file described in the section [Architecture](#Architecture).

## Usage

Create the rigidbodies for the hand and the drone in OptiTrack Motive and name them 'hand' and 'cf1' respectively. Make sure that the front of the drone is aligned with the front (x-axis) of the created rigidbody.

Make sure that Motive is streaming the data and that VRPN streaming is enabled. Make sure that the up axis is set to 'Z', otherwise the drone will flip during takeoff.

Activate the ROS workspace:
```bash
# Move to the catkin workspace folder
cd ros_ws

# Activate the workspace
source devel/setup.bash
```

Make sure your crazyflie is well configured in ```ros_ws/src/crazyswarm/launch/allCrazyflies.yaml```. Place the drone on the floor, making sure that the front of the drone is pointing towards the 'X-axis' of the OptiTrack referential, otherwise the drone will flip.

Execute the launch file:
```bash
roslaunch crazyswarm bidirectionalInterface.launch
```

## Architecture

The relevant files are the following:
* [handInterface.py](ros_ws/src/crazyswarm/scripts/handInterface.py), node translating hand motion to drone commands.
* [hapticFeedback.py](ros_ws/src/crazyswarm/scripts/hapticFeedback.py), node reading distance sensor values and sending commands to vibrating motors.
* [bidirectionalInterface.launch](ros_ws/src/crazyswarm/launch/bidirectionalInterface.launch), launch file intiating all necessary ROS nodes.

## Author:

* Thomas Havy
